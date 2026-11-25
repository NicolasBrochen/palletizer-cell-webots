"""Automate de cellule (superviseur) - palettisation robotisée.

Deux parties bien séparées :
  * PARTIE OPÉRATIVE SIMULÉE (« plant ») : déplacement des colis sur les bandes, butée de sas,
    transferts de palettes sur les rouleaux, banderoleuse, chariot élévateur.
  * PARTIE COMMANDE (« automate ») : décisions prises À PARTIR DES CAPTEURS Webots
    (cellules B1..B5, présence palettes S_P*, barrières immatérielles LC1/LC2 + muting,
    scrutateur laser, porte, AU) + dialogue avec le contrôleur robot.

Clavier (cliquer dans la vue 3D) :
  E : arrêt d'urgence (bascule)      D : ouvrir / fermer la porte d'accès
  R : réarmement                     P : faire entrer / sortir un piéton (zone chariot / sortie)
  S : arrêt fin de cycle / redémarrage
"""
import json
import math
import sys

from controller import DistanceSensor, Supervisor

# ------------------------------------------------------------------ géométrie (cf. tools/gen_world.py)
CONV_Y, CONV_Z = -1.70, 0.75
BOX = (0.40, 0.60, 0.30)              # colis : long. convoyeur x larg. x haut.
X_SPAWN = -7.75
X_SE = -0.45                           # butée d'entrée de sas
X_END = 0.40                           # butée fixe
V1, V2 = 0.25, 0.50                    # vitesse bande entrée / bande sas (création d'écart)
PAL_X, PAL_Z, PAL_H = 1.80, 0.10, 0.144
ST = {"P0": 2.4, "P1": 0.0, "P2": -2.8, "P3": -5.4}
ORDER = ["P0", "P1", "P2", "P3"]
LAYERS, LOTS_PER_LAYER = 4, 2
LOT_HALF_Y = [0.3, -0.3]               # demi-palette nord puis sud (le robot vient du sud)
V_PAL = 0.22
WRAP_OMEGA = 1.3                       # rad/s (~12 tr/min, plateau tournant)
FORK_D = 1.52                          # centre palette devant le centre du chariot
FORK_LIFT = 0.03
WAIT_POSE = (1.8, -9.5, math.pi / 2)
AISLE_Y = -9.5
SPOTS = [8.0 + 1.15 * i for i in range(6)]   # les 2 derniers sont déjà occupés (décor)
SPOT_Y = -12.6
SCAN_FIELD = [(0.3, 1.10, -6.8, -4.5), (2.5, 3.5, -6.8, -4.5)]
PED_PATH = "6.0 -7.2, 3.0 -7.2, 2.9 -4.7, 2.0 -3.95, 2.9 -4.7, 3.0 -7.2"


def rot_z(a):
    return [0, 0, 1, a]


class Load:
    """Palette + colis + film, déplacés comme un ensemble rigide."""
    def __init__(self, idx, pallet):
        self.idx, self.pallet = idx, pallet
        self.items = []                 # (node, dx, dy, dz, yaw0) dans le repère palette
        self.film = None
        self.film_h = 0.0
        self.x, self.y, self.z, self.th = PAL_X, ST["P0"], PAL_Z, 0.0
        self.station, self.target = "P0", None
        self.lots = 0
        self.state = "vide"

    def place(self):
        c, s = math.cos(self.th), math.sin(self.th)
        self.pallet.getField("translation").setSFVec3f([self.x, self.y, self.z])
        self.pallet.getField("rotation").setSFRotation(rot_z(math.pi / 2 + self.th))
        for n, dx, dy, dz, y0 in self.items:
            n.getField("translation").setSFVec3f([self.x + c * dx - s * dy, self.y + s * dx + c * dy, self.z + dz])
            n.getField("rotation").setSFRotation(rot_z(self.th + y0))
        if self.film is not None:
            self.film.getField("translation").setSFVec3f([self.x, self.y, self.z + 0.01 + self.film_h / 2])
            self.film.getField("rotation").setSFRotation(rot_z(self.th))


class CellPLC(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt_ms = int(self.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0
        self.root = self.getRoot().getField("children")
        self.ds = {}
        for i in range(self.getNumberOfDevices()):
            d = self.getDeviceByIndex(i)
            if isinstance(d, DistanceSensor):
                d.enable(self.dt_ms)
                self.ds[d.getName()] = d
        self.lidars = []
        for nm, px, py, yaw in (("scrutateur_laser_est", 3.75, -4.40, -2.3562),
                                ("scrutateur_laser_ouest", -0.15, -4.40, -0.7854)):
            rays = []
            for k in range(31):
                a = yaw + 1.65 / 2 - k * 1.65 / 30
                rays.append((self.ds["%s_%02d" % (nm, k)], a))
            self.lidars.append((rays, px, py, yaw))
        # --sans-ihm : pas de surimpression ni d'écran (enregistrement des vidéos du README)
        self.show_hmi = "--sans-ihm" not in sys.argv
        self.display = self.getDevice("IHM") if self.show_hmi else None
        self.tx = self.getDevice("plc_tx")
        self.rx = self.getDevice("robot_rx")
        self.rx.enable(self.dt_ms)
        self.kb = self.getKeyboard()
        self.kb.enable(self.dt_ms)
        # noeuds
        g = self.getFromDef
        self.n_butee = g("BUTEE_SAS")
        self.n_plateau = g("PLATEAU")
        self.n_chariot = g("CHARIOT_FILM")
        self.n_chariot_x = self.n_chariot.getField("translation").getSFVec3f()[0]
        self.belts = [(g("TT_C1"), V1, 7.53, 0.0), (g("TT_C2"), V2, 0.98, 0.0)]
        self.n_fork = g("FENWICK")
        self.n_porte = g("PORTE")
        self.lamps = {k: g(k) for k in ("FEU1_G", "FEU1_O", "FEU1_R", "FEU2_G", "FEU2_O", "FEU2_R",
                                         "LC1_LAMPE_MUTING", "LC2_LAMPE_MUTING", "LED_PORTE")}
        self.beams = {"LC1": g("LC1_FAISCEAUX"), "LC2": g("LC2_FAISCEAUX")}
        # état partie opérative
        self.boxes = []                 # dicts {def,node,x,state}
        self.box_count = 0
        self.spawn_timer = 0.0
        self.se_closed = False
        self.se_z = 0.72
        self.loads = []
        self.load_count = 0
        self.mag_n = 10
        self.mag_timer = 0.0
        self.wrap = None
        self.fork = dict(x=WAIT_POSE[0], y=WAIT_POSE[1], h=WAIT_POSE[2], plan=[], load=None, state="attente")
        self.spot_used = [False] * len(SPOTS)
        self.stored = []
        self.door_angle = 0.0
        self.ped = None
        # état partie commande
        self.estop = False
        self.door_open = False
        self.fault = None               # défaut sécurité mémorisé (nécessite réarmement)
        self.cycle_stop = False
        self.robot_ready = False
        self.robot_busy = False
        self.job_id = 0
        self.job = None
        self.lot_count = 0
        self.prev = {}
        self.b2_fall_t = None
        self.muting = False
        self.muting_t = 0.0
        self.scan_intrusion = False
        self.scan_clear_t = 0.0
        self.fork_alert = "OK"
        self.log_lines = []
        self.t_start = None
        self.n_palettes_done = 0

    # ================================================================ utilitaires capteurs
    def blocked(self, name):
        v = self.ds[name].getValue()
        if name.startswith("LC") or name.startswith("M"):
            return v < 1.10
        return v < 0.6

    def edge(self, name, rising=True):
        cur = self.blocked(name)
        old = self.prev.get(name, cur)
        self.prev[name] = cur
        return (cur and not old) if rising else (old and not cur)

    def log(self, msg):
        line = "[%7.1f s] %s" % (self.getTime(), msg)
        print(line)
        self.log_lines = (self.log_lines + [line])[-2:]

    def send(self, **m):
        self.tx.send(json.dumps(m))

    def set_lamp(self, key, on, color):
        n = self.lamps.get(key)
        if n:
            n.getField("emissiveColor").setSFColor(color if on else [0, 0, 0])

    # ================================================================ sécurité
    def safety_inputs(self):
        lc1 = any(self.blocked("LC1_faisceau%d" % i) for i in range(1, 10))
        lc2 = any(self.blocked("LC2_faisceau%d" % i) for i in range(1, 10))
        # muting LC1 : séquence M1 -> M2 pendant un transfert sortant, durée max 20 s
        m1, m2 = self.blocked("M1_muting"), self.blocked("M2_muting")
        exiting = any(l.target == "P3" for l in self.loads)
        if not self.muting and exiting and m1 and not m2:
            self.muting, self.muting_t = True, 0.0
            self.log("LC1 muting on (pallet leaving the cell)")
        if self.muting:
            self.muting_t += self.dt
            if (not m1 and not m2) or self.muting_t > 20.0:
                self.muting = False
                self.log("LC1 muting off")
        self.set_lamp("LC1_LAMPE_MUTING", self.muting, [1, 1, 1])
        if self.fault is None:
            if self.estop:
                self.trip("EMERGENCY STOP pressed")
            elif self.door_open:
                self.trip("Access door open (interlock)")
            elif lc1 and not self.muting:
                self.trip("Light curtain LC1 (pallet exit) interrupted")
            elif lc2:
                self.trip("Light curtain LC2 (pallet supply) interrupted")
        self.lc_state = (lc1, lc2)
        self.set_lamp("LED_PORTE", not self.door_open, [0, 0.8, 0])

    def trip(self, why):
        self.fault = why
        self.send(cmd="hold")
        self.set_conveyors(False)
        self.log("SAFETY STOP: " + why)

    def reset(self):
        lc1, lc2 = self.lc_state
        if self.fault is None:
            return
        if self.estop or self.door_open or (lc1 and not self.muting) or lc2:
            self.log("Reset refused: safety condition still active")
            return
        self.log("Reset OK - cycle resumed")
        self.fault = None
        self.send(cmd="release_hold")
        self.set_conveyors(True)

    def scanner(self):
        """Scrutateur laser devant le poste d'évacuation (champs latéraux)."""
        hits = 0
        for ld, px, py, yaw in self.lidars:
            hits += self._scan_hits(ld, px, py, yaw)
        self._scan_update(hits)

    def _scan_hits(self, rays, px, py, yaw):
        """Scrutateur émulé par un éventail de 31 faisceaux (pas 3.2°, portée 8 m)."""
        hits = 0
        for ds, a in rays:
            r = ds.getValue()
            if r >= 7.9:
                continue
            x, y = px + r * math.cos(a), py + r * math.sin(a)
            for (x0, x1, y0, y1) in SCAN_FIELD:
                if x0 <= x <= x1 and y0 <= y <= y1:
                    hits += 1
        return hits

    def _scan_update(self, hits):
        if hits >= 1:
            if not self.scan_intrusion:
                self.log("Laser scanner: person detected in the exit zone")
            self.scan_intrusion = True
            self.scan_clear_t = 0.0
        elif self.scan_intrusion:
            self.scan_clear_t += 2 * self.dt
            if self.scan_clear_t > 2.0:
                self.scan_intrusion = False
                self.log("Laser scanner: exit zone clear")

    def running(self):
        return self.fault is None

    # ================================================================ convoyeurs colis
    def set_conveyors(self, on):
        """Les bandes sont animées (texture) uniquement quand la cellule est en marche (cf. plant_boxes)."""
        self.belts_on = on

    def animate_belts(self):
        for i, (tt, v, L, off) in enumerate(self.belts):
            off = (off - v * self.dt / L) % 1.0
            self.belts[i] = (tt, v, L, off)
            tt.getField("translation").setSFVec2f([off, 0])

    def obtain(self, d, vrml, pos, rot=None):
        """Réutilise le noeud DEF `d` s'il existe déjà (réservoir pré-créé pour l'export HTML5), sinon l'importe."""
        n = self.getFromDef(d)
        if n is None:
            self.root.importMFNodeFromString(-1, vrml)
            return self.getFromDef(d)
        n.getField("translation").setSFVec3f(list(pos))
        if rot is not None:
            n.getField("rotation").setSFRotation(rot)
        return n

    def spawn_box(self):
        self.box_count += 1
        d = "COLIS_%d" % self.box_count
        pos = (X_SPAWN, CONV_Y, CONV_Z + BOX[2] / 2)
        node = self.obtain(d, 'DEF %s CardboardBox { translation %g %g %g name "colis %d" size %g %g %g }'
                           % (d, *pos, self.box_count, *BOX), pos, [0, 0, 1, 0])
        self.boxes.append(dict(d=d, node=node, x=X_SPAWN))

    def plant_boxes(self):
        if not self.running():
            return
        self.animate_belts()
        hx = BOX[0] / 2
        lead = None
        for b in sorted(self.boxes, key=lambda b: -b["x"]):
            v = V2 if b["x"] > X_SE else V1
            lim = X_END - hx
            if lead is not None:
                lim = min(lim, lead["x"] - BOX[0] - 0.005)
            if self.se_z > 0.80 and b["x"] + hx <= X_SE + 0.01:
                lim = min(lim, X_SE - 0.015 - hx)
            nx = min(b["x"] + v * self.dt, max(b["x"], lim))
            if nx != b["x"]:
                b["x"] = nx
                b["node"].getField("translation").setSFVec3f([nx, CONV_Y, CONV_Z + BOX[2] / 2])
            lead = b
        # vérin de butée (course 0.25 m en 0.3 s)
        tgt = 0.95 if self.se_closed else 0.70
        if abs(self.se_z - tgt) > 1e-3:
            self.se_z += max(-0.8 * self.dt, min(0.8 * self.dt, tgt - self.se_z))
            self.n_butee.getField("translation").setSFVec3f([0, 0, self.se_z])

    def logic_infeed(self):
        if not self.running():
            return
        # arrivée colis : production amont simulée, B1 annonce la présence
        self.spawn_timer += self.dt
        if self.edge("B1_annonce_colis"):
            self.log("B1: case announced at conveyor infeed")
        accu_full = self.blocked("B5_accumulation") and self.prev.get("B5_t", 0) > 3.0
        self.prev["B5_t"] = self.prev.get("B5_t", 0) + self.dt if self.blocked("B5_accumulation") else 0
        if (self.spawn_timer > 4.0 and not self.blocked("B1_annonce_colis") and not accu_full
                and not self.cycle_stop):
            if all(b["x"] > X_SPAWN + BOX[0] + 0.25 for b in self.boxes):
                self.spawn_box()
                self.spawn_timer = 0.0
        # formation du lot dans le sas : comptage B2, fermeture butée après le 2e colis
        cur = self.blocked("B2_comptage_sas")
        old = self.prev.get("B2", cur)
        self.prev["B2"] = cur
        if not self.se_closed:
            if cur and not old:
                self.lot_count += 1
            if old and not cur and self.lot_count >= 2:
                self.b2_fall_t = 0.0
            if self.b2_fall_t is not None:
                self.b2_fall_t += self.dt
                if self.b2_fall_t > 0.3:
                    self.se_closed = True
                    self.b2_fall_t = None
                    self.log("Buffer: 2 cases counted, infeed stop raised")

    def lot_ready(self):
        if not (self.se_closed and self.lot_count >= 2 and self.blocked("B3_lot_colis1")
                and self.blocked("B4_lot_colis2")):
            return None
        lot = [b for b in self.boxes if b["x"] > X_SE]
        if len(lot) != 2 or any(abs(b["x"] - (X_END - BOX[0] * (0.5 + i))) > 0.02
                                for i, b in enumerate(sorted(lot, key=lambda b: -b["x"]))):
            return None
        return lot

    # ================================================================ flux palettes
    def station_load(self, st):
        for l in self.loads:
            if l.station == st or l.target == st:
                return l
        return None

    def dispense_pallet(self):
        if self.station_load("P0") is not None:
            self.mag_timer = 0.0
            return
        self.mag_timer += self.dt
        if self.mag_timer < 2.0:
            return
        self.load_count += 1
        d = "PALETTE_%d" % self.load_count
        pos = (PAL_X, ST["P0"], PAL_Z)
        node = self.obtain(d, 'DEF %s WoodenPallet { translation %g %g %g rotation 0 0 1 1.5708 name "palette %d" }'
                           % (d, *pos, self.load_count), pos, [0, 0, 1, 1.5708])
        self.loads.append(Load(self.load_count, node))
        self.mag_n -= 1
        if self.mag_n <= 1:
            self.mag_n = 10
            self.log("Empty pallet magazine refilled (10 pallets)")
        self.log("Empty pallet #%d dispensed by the magazine" % self.load_count)

    def plant_pallets(self):
        if not self.running():
            return
        for l in self.loads:
            if l.target is None:
                continue
            ty = ST[l.target]
            # le transfert vers l'évacuation est suspendu si le scrutateur voit quelqu'un
            if l.target == "P3" and self.scan_intrusion:
                continue
            step = V_PAL * self.dt
            if abs(ty - l.y) <= step:
                l.y = ty
                l.station, l.target = l.target, None
            else:
                l.y += step if ty > l.y else -step
            l.place()

    def logic_pallets(self):
        if not self.running():
            return
        self.dispense_pallet()
        for l in self.loads:
            if l.target is not None or l.station not in ORDER:
                continue
            nxt = None
            if l.station == "P0" and l.state == "vide" and not self.cycle_stop:
                nxt = "P1"
            elif l.station == "P1" and l.state == "pleine" and not self.robot_busy:
                nxt = "P2"
            elif l.station == "P2" and l.state == "banderolee":
                nxt = "P3"
            if nxt and self.station_load(nxt) is None and not (nxt == "P3" and self.scan_intrusion):
                l.target = nxt
                l.station = "transfert"
                self.log("Pallet #%d: transfer to %s" % (l.idx, {"P1": "palletizing station",
                         "P2": "stretch wrapper", "P3": "exit station"}[nxt]))
        # palette arrivée en P2 -> banderolage
        l2 = self.station_load("P2")
        if l2 and l2.station == "P2" and l2.state == "pleine" and self.wrap is None:
            if self.blocked("S_P2_banderoleuse"):
                self.start_wrap(l2)

    # ================================================================ robot
    def logic_robot(self):
        while self.rx.getQueueLength() > 0:
            m = json.loads(self.rx.getString())
            self.rx.nextPacket()
            e = m.get("evt")
            if e == "ready":
                self.robot_ready = True
                self.log("Robot ready (home position)")
            elif e == "picked":
                for b in self.job["lot"]:
                    self.boxes.remove(b)
                # un colis déjà présent devant B2 (en attente contre la butée) compte pour le lot suivant
                self.lot_count = 1 if self.blocked("B2_comptage_sas") else 0
                self.se_closed = False
                self.log("Lot #%d picked - buffer reopened" % self.job["id"])
            elif e == "placed":
                l = self.job["load"]
                for b in self.job["lot"]:
                    p = b["node"].getPosition()
                    l.items.append((b["node"], p[0] - l.x, p[1] - l.y, p[2] - l.z, 0.0))
                l.lots += 1
                self.log("Lot placed: pallet #%d layer %d/%d (%d/%d lots)" % (
                    l.idx, (l.lots - 1) // LOTS_PER_LAYER + 1, LAYERS, l.lots, LAYERS * LOTS_PER_LAYER))
                if l.lots >= LAYERS * LOTS_PER_LAYER:
                    l.state = "pleine"
                    self.log("Pallet #%d complete (%d cases)" % (l.idx, 2 * l.lots))
            elif e == "done":
                self.robot_busy = False
                self.job = None
            elif e == "error":
                self.log("ROBOT: " + m.get("msg", ""))
        if not self.running() or not self.robot_ready or self.robot_busy:
            return
        l = self.station_load("P1")
        if l is None or l.station != "P1" or l.state == "pleine" or not self.blocked("S_P1_palettisation"):
            return
        lot = self.lot_ready()
        if lot is None:
            return
        k = l.lots
        layer, half = k // LOTS_PER_LAYER, k % LOTS_PER_LAYER
        top = PAL_Z + PAL_H + BOX[2] * (layer + 1)
        px = sum(b["x"] for b in lot) / 2
        self.job_id += 1
        self.job = dict(id=self.job_id, lot=lot, load=l)
        l.state = "en cours"
        self.robot_busy = True
        self.send(cmd="pick_place", id=self.job_id, boxes=[b["d"] for b in lot],
                  pick=[px, CONV_Y, CONV_Z + BOX[2], 0.0],
                  place=[l.x, l.y + LOT_HALF_Y[half], top, 0.0],
                  clear_z=max(1.45, top + 0.15), box_h=BOX[2])
        self.log("Robot order: lot #%d -> pallet #%d layer %d %s" % (
            self.job_id, l.idx, layer + 1, "north" if half == 0 else "south"))

    # ================================================================ banderoleuse
    def start_wrap(self, l):
        top = PAL_H + BOX[2] * LAYERS
        d = "FILM_%d" % l.idx
        pos = (l.x, l.y, l.z + 0.035)
        l.film = self.obtain(d, 'DEF %s Transform { translation %g %g %g scale 1 1 0.05 children [ Shape { appearance '
                             'PBRAppearance { baseColor 0.85 0.9 0.95 transparency 0.55 roughness 0.1 metalness 0 } '
                             'geometry Box { size 0.82 1.22 1 } } ] }' % (d, *pos), pos)
        self.wrap = dict(load=l, t=0.0, ang=0.0, top=top, zc=PAL_Z + 0.25)
        self.log("Wrapping pallet #%d started" % l.idx)

    def plant_wrap(self):
        """Banderoleuse à plateau tournant : la charge tourne, le chariot porte-bobine monte puis redescend."""
        if self.wrap is None or not self.running():
            return
        w = self.wrap
        l = w["load"]
        w["t"] += self.dt
        T_UP, T_TOP = 16.0, 4.0
        z_low, z_top = PAL_Z + 0.25, PAL_Z + w["top"] - 0.15
        if w["t"] < 2.0:                                  # tours de pied (bas de palette)
            w["zc"] = z_low
        elif w["t"] < 2.0 + T_UP:
            w["zc"] = z_low + (z_top - z_low) * (w["t"] - 2.0) / T_UP
        elif w["t"] < 2.0 + T_UP + T_TOP:                 # tours de tête
            w["zc"] = z_top
        phase_end = w["t"] >= 2.0 + T_UP + T_TOP
        if not phase_end:
            w["ang"] += WRAP_OMEGA * self.dt
            film_h = max(0.05, min(w["top"] + 0.02, w["zc"] + 0.25 - PAL_Z))
            if abs(film_h - l.film_h) > 1e-3:
                l.film_h = film_h
                l.film.getField("scale").setSFVec3f([1, 1, film_h])
        else:
            # arrêt plateau indexé (multiple de 2π) pour réaligner les chaînes, puis retour chariot en bas
            a = w["ang"] % (2 * math.pi)
            if a > 1e-3:
                w["ang"] += min(WRAP_OMEGA * self.dt, 2 * math.pi - a)
            else:
                w["ang"] = 0.0
                w["zc"] = max(z_low, w["zc"] - 0.5 * self.dt)
                if w["zc"] <= z_low + 1e-3:
                    l.state = "banderolee"
                    self.wrap = None
                    self.log("Wrapping pallet #%d done" % l.idx)
        l.th = w["ang"] if self.wrap else 0.0
        l.place()
        self.n_plateau.getField("rotation").setSFRotation(rot_z(l.th))
        self.n_chariot.getField("translation").setSFVec3f([self.n_chariot_x, 0, w["zc"]])

    # ================================================================ chariot élévateur
    def fork_plan(self, spot):
        sx = SPOTS[spot]
        pick_y = ST["P3"] - FORK_D
        return [
            ("go", 1.8, pick_y, math.pi / 2, 1, 0.6),
            ("lift",),
            ("go", 1.8, AISLE_Y, math.pi / 2, -1, 1.2),
            ("turn", 0.0),
            ("go", sx, AISLE_Y, 0.0, 1, 2.0),
            ("turn", -math.pi / 2),
            ("go", sx, SPOT_Y + FORK_D, -math.pi / 2, 1, 0.8),
            ("drop", spot),
            ("go", sx, AISLE_Y, -math.pi / 2, -1, 1.2),
            ("turn", math.pi),
            ("go", WAIT_POSE[0], AISLE_Y, math.pi, 1, 2.0),
            ("turn", math.pi / 2),
            ("end",),
        ]

    def pedestrian_near_fork(self):
        if self.ped is None:
            return 99.0
        p = self.ped.getPosition()
        return math.hypot(p[0] - self.fork["x"], p[1] - self.fork["y"])

    def plant_fork(self):
        fk = self.fork
        # détection piétons embarquée (type caméra IA / anticollision) : arrêt < 3.5 m, ralenti < 6 m
        dist = self.pedestrian_near_fork()
        self.fork_alert = "STOP" if dist < 3.5 else ("RALENTI" if dist < 6.0 else "OK")
        if fk["state"] == "attente":
            l3 = self.station_load("P3")
            if (l3 and l3.station == "P3" and l3.state == "banderolee" and self.blocked("S_P3_evacuation")
                    and not self.scan_intrusion):
                free = [i for i, u in enumerate(self.spot_used) if not u]
                if not free:   # expédition : on libère le plus ancien emplacement
                    old = self.stored.pop(0)
                    self.remove_load(old)
                    free = [i for i, u in enumerate(self.spot_used) if not u]
                    self.log("Storage full: oldest pallet shipped")
                fk["plan"] = self.fork_plan(free[0])
                fk["load"] = l3
                fk["state"] = "mission"
                self.log("Forklift: mission pallet #%d -> storage slot %d" % (l3.idx, free[0] + 1))
            return
        if self.fork_alert == "STOP":
            return
        k = 0.35 if self.fork_alert == "RALENTI" else 1.0
        act = fk["plan"][0]
        done = False
        if act[0] == "go":
            _, tx, ty, th, sens, vmax = act
            dx, dy = tx - fk["x"], ty - fk["y"]
            d = math.hypot(dx, dy)
            v = min(vmax, 0.25 + 0.8 * d) * k
            if d <= v * self.dt:
                fk["x"], fk["y"] = tx, ty
                done = True
            else:
                fk["x"] += dx / d * v * self.dt
                fk["y"] += dy / d * v * self.dt
            fk["h"] = th
        elif act[0] == "turn":
            dh = (act[1] - fk["h"] + math.pi) % (2 * math.pi) - math.pi
            st = 0.7 * k * self.dt
            if abs(dh) <= st:
                fk["h"] = act[1]
                done = True
            else:
                fk["h"] += st if dh > 0 else -st
        elif act[0] == "lift":
            l = fk["load"]
            l.z = PAL_Z + FORK_LIFT
            l.station = "chariot"
            l.state = "transport"
            self.log("Forklift: pallet #%d lifted" % l.idx)
            done = True
        elif act[0] == "drop":
            l = fk["load"]
            l.z = PAL_Z
            l.place()
            l.station, l.state = "hangar", "stockee"
            self.spot_used[act[1]] = True
            l.spot = act[1]
            self.stored.append(l)
            self.n_palettes_done += 1
            self.log("Forklift: pallet #%d stored (slot %d)" % (l.idx, act[1] + 1))
            fk["load"] = None
            done = True
        elif act[0] == "end":
            fk["state"] = "attente"
            done = True
        if done:
            fk["plan"].pop(0)
        self.n_fork.getField("translation").setSFVec3f([fk["x"], fk["y"], 0.81])
        self.n_fork.getField("rotation").setSFRotation(rot_z(fk["h"]))
        if fk["load"] is not None and fk["load"].station == "chariot":
            l = fk["load"]
            l.x = fk["x"] + FORK_D * math.cos(fk["h"])
            l.y = fk["y"] + FORK_D * math.sin(fk["h"])
            l.th = fk["h"] - math.pi / 2
            l.place()

    def remove_load(self, l):
        for n, *_ in l.items:
            n.remove()
        if l.film:
            l.film.remove()
        l.pallet.remove()
        self.spot_used[l.spot] = False
        self.loads.remove(l)

    # ================================================================ IHM / clavier / signalisation
    def handle_keys(self):
        keys = []
        k = self.kb.getKey()
        while k != -1:
            keys.append(chr(k & 0xFF).upper() if (k & 0xFF) < 128 else "")
            k = self.kb.getKey()
        # commandes à distance : écrire p.ex. "R" dans le champ customData du robot « automate cellule »
        cd = self.getCustomData()
        if cd:
            keys += list(cd.upper())
            self.setCustomData("")
        for c in keys:
            if c == "E":
                self.estop = not self.estop
                self.log("Emergency stop %s" % ("PRESSED" if self.estop else "released"))
            elif c == "D":
                self.door_open = not self.door_open
                self.log("Access door %s" % ("open" if self.door_open else "closed"))
            elif c == "R":
                self.reset()
            elif c == "S":
                self.cycle_stop = not self.cycle_stop
                self.log("End-of-cycle stop requested" if self.cycle_stop else "Production restarted")
            elif c == "P":
                self.toggle_pedestrian()

    def toggle_pedestrian(self):
        if self.ped is None:
            self.root.importMFNodeFromString(-1, 'DEF PIETON Pedestrian { translation 6 -7.2 1.27 name "pieton" '
                                             'controllerArgs [ "--trajectory=%s" "--speed=0.9" ] '
                                             'shirtColor 1 0.55 0 enableBoundingObject TRUE }' % PED_PATH)
            self.ped = self.getFromDef("PIETON")
            self.log("Pedestrian entered the forklift / exit zone")
        else:
            self.ped.remove()
            self.ped = None
            self.log("Pedestrian left")

    def plant_door(self):
        tgt = math.pi / 2 if self.door_open else 0.0
        if abs(self.door_angle - tgt) > 1e-3:
            self.door_angle += max(-1.5 * self.dt, min(1.5 * self.dt, tgt - self.door_angle))
            self.n_porte.getField("rotation").setSFRotation(rot_z(self.door_angle))

    def signals(self):
        t = self.getTime()
        blink = int(t * 2) % 2 == 0
        red = self.fault is not None
        orange = (not red) and (self.muting or self.scan_intrusion or self.fork_alert != "OK" or self.cycle_stop)
        green = (not red) and not self.cycle_stop
        for p in ("FEU1", "FEU2"):
            self.set_lamp(p + "_R", red and (blink or self.estop), [1, 0, 0])
            self.set_lamp(p + "_O", orange and blink, [1, 0.55, 0])
            self.set_lamp(p + "_G", green, [0, 1, 0])
        # faisceaux visibles uniquement s'ils ne sont pas inhibés
        lc1, lc2 = self.lc_state
        self.beams["LC1"].getField("translation").setSFVec3f([0, 0, -10 if self.muting else 0])

    def hmi(self):
        st = "SAFETY STOP" if self.fault else ("END-OF-CYCLE STOP" if self.cycle_stop else "AUTO - PRODUCTION")
        l1 = self.station_load("P1")
        pal_txt = "P1: pallet #%d  %d/%d lots" % (l1.idx, l1.lots, LAYERS * LOTS_PER_LAYER) if l1 else "P1: waiting for pallet"
        # surimpression réduite au journal : les 2 derniers événements (en jaune, en bas à gauche)
        for i, s in enumerate(self.log_lines):
            self.setLabel(i, s, 0.01, 0.83 + 0.035 * i, 0.06, 0xFFE080, 0.0, "Arial")
        # écran du pupitre
        d = self.display
        d.setColor(0x202830)
        d.fillRectangle(0, 0, 320, 200)
        d.setColor(0xD03030 if self.fault else 0x30A040)
        d.fillRectangle(0, 0, 320, 28)
        d.setColor(0xFFFFFF)
        d.setFont("Arial", 14, True)
        d.drawText(st, 8, 6)
        d.setFont("Arial", 11, True)
        yy = 36
        for s in [pal_txt, "Buffer: %d cases" % self.lot_count, "Robot: %s" % ("cycle" if self.robot_busy else "idle"),
                  "Pallets stored: %d" % self.n_palettes_done, "Forklift: %s" % self.fork["state"],
                  (self.fault or "Safety OK")[:44]]:
            d.drawText(s, 8, yy)
            yy += 22

    # ================================================================ boucle principale
    def run(self):
        self.step(self.dt_ms)
        self.lc_state = (False, False)
        self.log("Cell PLC started")
        self.send(cmd="speed", value=1.0)
        frame = 0
        while self.step(self.dt_ms) != -1:
            frame += 1
            self.handle_keys()
            self.safety_inputs()
            if frame % 2 == 0:
                self.scanner()
            # partie commande
            self.logic_infeed()
            self.logic_robot()
            self.logic_pallets()
            # partie opérative
            self.plant_boxes()
            self.plant_pallets()
            self.plant_wrap()
            self.plant_fork()      # hors cellule : protégé par sa propre détection piétons
            self.plant_door()
            if frame % 4 == 0:
                self.signals()
                if self.show_hmi:
                    self.hmi()


CellPLC().run()
