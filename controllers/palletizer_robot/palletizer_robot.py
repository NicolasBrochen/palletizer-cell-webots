"""Contrôleur du robot palettiseur ABB IRB 4600-40 + préhenseur à ventouses.

Reçoit les ordres de l'automate de cellule (cell_plc) par radio (Receiver canal 1)
et renvoie ses événements (Emitter canal 2). Messages JSON :

  PLC -> robot : {"cmd": "pick_place", "id": n, "boxes": [DEF...],
                  "pick": [x, y, z, yaw], "place": [x, y, z, yaw], "clear_z": z}
                 {"cmd": "hold"}  / {"cmd": "release_hold"}   (arrêt de sécurité cat. 2 / reprise)
                 {"cmd": "speed", "value": 0..1}              (override vitesse)
  robot -> PLC : {"evt": "ready"}, {"evt": "gripped", "id"}, {"evt": "picked", "id"},
                 {"evt": "placed", "id", "boxes"}, {"evt": "done", "id"},
                 {"evt": "vacuum", "ok": bool}, {"evt": "error", "msg"}

Le préhenseur à ventouses est simulé ainsi : quand le vide est établi
(vacuostat OK + détecteur de présence produit), les colis sont solidarisés
cinématiquement au TCP (le contrôleur est superviseur).
"""
import json
import math

from controller import Supervisor

from irb4600_kin import ik_down

TOOL_LEN = 0.30          # bride -> face des ventouses (m)
HOME_TCP = (0.9, -1.2, 1.55, 0.0)
JOINT_SPEED = 1.1        # rad/s moyen en interpolation articulaire
LIN_SPEED = 0.6          # m/s en mouvement linéaire
APPROACH_SPEED = 0.25    # m/s en accostage final


# ------------------------------------------------------------------ maths utilitaires
def mat_from_list(o):
    return [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]


def mat_mul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_T(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(m):
    c = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    ang = math.acos(c)
    if ang < 1e-6:
        return [0, 0, 1, 0]
    if abs(ang - math.pi) < 1e-4:
        xx = math.sqrt(max(0, (m[0][0] + 1) / 2))
        yy = math.sqrt(max(0, (m[1][1] + 1) / 2))
        zz = math.sqrt(max(0, (m[2][2] + 1) / 2))
        if xx > 1e-3:
            yy = math.copysign(yy, m[0][1]); zz = math.copysign(zz, m[0][2])
        elif yy > 1e-3:
            zz = math.copysign(zz, m[1][2])
        return [xx, yy, zz, ang]
    s = 2 * math.sin(ang)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, ang]


def quintic(s):
    s = max(0.0, min(1.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


# ------------------------------------------------------------------ segments de trajectoire
class JointSeg:
    def __init__(self, q0, q1, speed_scale=1.0):
        self.q0, self.q1 = list(q0), list(q1)
        dq = max(abs(a - b) for a, b in zip(q0, q1))
        self.T = max(0.6, dq / (JOINT_SPEED * speed_scale))
        self.t = 0.0

    def sample(self):
        s = quintic(self.t / self.T)
        return [a + (b - a) * s for a, b in zip(self.q0, self.q1)]


class LinSeg:
    def __init__(self, p0, p1, speed, q_ref):
        self.p0, self.p1 = list(p0), list(p1)
        d = math.dist(p0[:3], p1[:3])
        self.T = max(0.4, 1.875 * d / speed)
        self.t = 0.0
        self.q_ref = q_ref

    def sample(self):
        s = quintic(self.t / self.T)
        p = [a + (b - a) * s for a, b in zip(self.p0, self.p1)]
        q = ik_down(p[0], p[1], p[2], p[3], TOOL_LEN, self.q_ref)
        if q is None:
            return self.q_ref
        self.q_ref = q
        return q


class Action:
    """Action ponctuelle exécutée à un instant de la séquence."""
    def __init__(self, fn, wait=0.0):
        self.fn, self.T, self.t, self.done = fn, wait, 0.0, False


class Palletizer(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        names = ["A", "B", "C", "D", "E", "F"]
        self.motors = [self.getDevice(n + " motor") for n in names]
        self.sensors = [self.getDevice(n + " sensor") for n in names]
        for s in self.sensors:
            s.enable(self.dt)
        self.rx = self.getDevice("plc_rx")
        self.rx.enable(self.dt)
        self.tx = self.getDevice("robot_tx")
        self.presence = self.getDevice("capteur_presence_colis")
        self.presence.enable(self.dt)
        self.tcp_node = self.getFromDef("TCP")
        self.base = self.getSelf()
        self.queue = []
        self.held = []         # [(node, rel_pos, rel_rot)]
        self.hold = False      # arrêt de sécurité (catégorie 2 : maintien sous asservissement)
        self.override = 1.0
        self.q_cmd = None
        self.job = None
        self.vacuum = False

    # -------------------------------------------------------------- communication
    def send(self, **msg):
        self.tx.send(json.dumps(msg))

    def poll(self):
        while self.rx.getQueueLength() > 0:
            msg = json.loads(self.rx.getString())
            self.rx.nextPacket()
            cmd = msg.get("cmd")
            if cmd == "hold":
                self.hold = True
            elif cmd == "release_hold":
                self.hold = False
            elif cmd == "speed":
                self.override = max(0.05, min(1.0, float(msg["value"])))
            elif cmd == "pick_place" and self.job is None:
                self.plan_job(msg)

    # -------------------------------------------------------------- base robot -> monde
    def world_to_base(self, p):
        bp = self.base.getPosition()
        R = mat_from_list(self.base.getOrientation())
        v = mat_vec(mat_T(R), [p[0] - bp[0], p[1] - bp[1], p[2] - bp[2]])
        yaw_base = math.atan2(R[1][0], R[0][0])
        return [v[0], v[1], v[2], p[3] - yaw_base]

    # -------------------------------------------------------------- préhenseur
    def tool_pose(self):
        if self.tcp_node is None:  # repli : premier noeud du handSlot + longueur outil
            g = self.base.getField("handSlot").getMFNode(0)
            R = mat_from_list(g.getOrientation())
            p = g.getPosition()
            off = mat_vec(R, [0, 0, TOOL_LEN])
            return [p[i] + off[i] for i in range(3)], R
        return self.tcp_node.getPosition(), mat_from_list(self.tcp_node.getOrientation())

    def grip(self, defs):
        if self.presence.getValue() > 0.2:
            self.send(evt="error", msg="No case detected under the gripper")
        P, R = self.tool_pose()
        Rt = mat_T(R)
        self.held = []
        for d in defs:
            n = self.getFromDef(d)
            if n is None:
                continue
            bp = n.getPosition()
            br = mat_from_list(n.getOrientation())
            rel_p = mat_vec(Rt, [bp[i] - P[i] for i in range(3)])
            rel_r = mat_mul3(Rt, br)
            self.held.append((n, rel_p, rel_r))
        self.vacuum = True
        self.send(evt="vacuum", ok=True)
        self.send(evt="gripped", id=self.job["id"])

    def release(self):
        placed = []
        for n, _, _ in self.held:
            # recalage fin : z exact posé, lacet arrondi au quart de tour
            p = n.getPosition()
            R = mat_from_list(n.getOrientation())
            yaw = math.atan2(R[1][0], R[0][0])
            yaw = round(yaw / (math.pi / 2)) * (math.pi / 2)
            n.getField("translation").setSFVec3f([p[0], p[1], self.job["place_box_z"]])
            n.getField("rotation").setSFRotation([0, 0, 1, yaw])
            placed.append(n.getDef())
        self.held = []
        self.vacuum = False
        self.send(evt="vacuum", ok=False)
        self.send(evt="placed", id=self.job["id"], boxes=placed)

    def carry(self):
        if not self.held:
            return
        P, R = self.tool_pose()
        for n, rp, rr in self.held:
            wp = mat_vec(R, rp)
            n.getField("translation").setSFVec3f([P[0] + wp[0], P[1] + wp[1], P[2] + wp[2]])
            n.getField("rotation").setSFRotation(axis_angle(mat_mul3(R, rr)))

    # -------------------------------------------------------------- planification
    def plan_job(self, msg):
        pick = self.world_to_base(msg["pick"])
        place = self.world_to_base(msg["place"])
        clear = msg.get("clear_z", 1.45)
        self.job = dict(id=msg["id"], boxes=msg["boxes"],
                        place_box_z=msg["place"][2] - msg.get("box_h", 0.3) / 2)
        q = self.q_cmd
        up_pick = [pick[0], pick[1], max(clear, pick[2] + 0.35), pick[3]]
        pre_pick = [pick[0], pick[1], pick[2] + 0.08, pick[3]]
        up_place = [place[0], place[1], max(clear, place[2] + 0.15), place[3]]
        pre_place = [place[0], place[1], place[2] + 0.06, place[3]]
        home = self.world_to_base(self.base_home_world())
        jid = self.job["id"]
        self.queue = [
            ("j", up_pick), ("l", pre_pick, LIN_SPEED), ("l", pick, APPROACH_SPEED),
            ("a", lambda: self.grip(self.job["boxes"]), 0.35),       # établissement du vide
            ("l", pre_pick, APPROACH_SPEED), ("l", up_pick, LIN_SPEED),
            ("a", lambda: self.send(evt="picked", id=jid), 0.0),
            ("j", up_place), ("l", pre_place, LIN_SPEED), ("l", place, APPROACH_SPEED),
            ("a", self.release, 0.3),                                 # soufflage / mise à l'air
            ("l", up_place, LIN_SPEED),
            ("j", home),
            ("a", self.finish_job, 0.0),
        ]
        self.cur = None

    def base_home_world(self):
        bp = self.base.getPosition()
        return [bp[0] + HOME_TCP[0], bp[1] + HOME_TCP[1], bp[2] + HOME_TCP[2], HOME_TCP[3]]

    def finish_job(self):
        self.send(evt="done", id=self.job["id"])
        self.job = None

    def next_segment(self):
        item = self.queue.pop(0)
        kind = item[0]
        if kind == "j":
            tgt = item[1]
            q1 = ik_down(tgt[0], tgt[1], tgt[2], tgt[3], TOOL_LEN, self.q_cmd)
            if q1 is None:
                self.send(evt="error", msg="Target out of reach %s" % tgt)
                q1 = self.q_cmd
            self.cur = JointSeg(self.q_cmd, q1, self.override)
            self.cur_tcp = tgt
        elif kind == "l":
            start = getattr(self, "cur_tcp", None) or item[1]
            self.cur = LinSeg(start, item[1], item[2] * self.override, self.q_cmd)
            self.cur_tcp = item[1]
        else:
            self.cur = Action(item[1], item[2])

    # -------------------------------------------------------------- boucle
    def run(self):
        self.step(self.dt)
        self.q_cmd = [s.getValue() for s in self.sensors]
        home = self.world_to_base(self.base_home_world())
        q_home = ik_down(*home, TOOL_LEN, self.q_cmd)
        self.queue = [("j", home)]
        self.cur = None
        self.job = None
        sent_ready = False
        dt = self.dt / 1000.0
        while self.step(self.dt) != -1:
            self.poll()
            if not self.hold:
                if self.cur is None and self.queue:
                    self.next_segment()
                if self.cur is not None:
                    self.cur.t += dt
                    if isinstance(self.cur, Action):
                        if not self.cur.done:
                            self.cur.fn()
                            self.cur.done = True
                    else:
                        self.q_cmd = self.cur.sample()
                    if self.cur.t >= self.cur.T:
                        if not isinstance(self.cur, Action):
                            self.q_cmd = self.cur.sample()
                        self.cur = None
            for m, q in zip(self.motors, self.q_cmd):
                m.setPosition(q)
            self.carry()
            if not sent_ready and self.cur is None and not self.queue:
                self.send(evt="ready")
                sent_ready = True


Palletizer().run()
