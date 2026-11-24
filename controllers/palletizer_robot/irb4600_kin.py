"""Cinématique directe / inverse analytique de l'ABB IRB 4600-40 (modèle Webots R2025a).

Les décalages sont relevés dans Irb4600-40.proto (chaîne de HingeJoint).
L'IK est spécialisée palettisation : outil toujours vertical (axe outil vers le bas),
q4 = 0, seul le lacet (yaw) de l'outil est libre -> robot "4 axes" comme un palettiseur.
Pas de dépendance (ni numpy, ni ikpy).
"""
import math

# Décalages de la chaîne (repère parent, m)
Z_BASE = 0.159498
J2_OFF = (0.178445, -0.122498, 0.334888)
J3_OFF = (-0.003447, -0.0267, 1.095594)
J4_OFF = (0.340095, 0.149198, 0.174998)
J5_OFF = (0.929888, 0.0, 0.0)
J6_OFF = (0.125, 0.0, 0.0)
FLANGE = 0.06  # handSlot à +0.06 m sur x de la liaison 6

LIMITS = [(-3.1415, 3.1415), (-1.5708, 2.61799), (-3.1415, 1.309),
          (-6.98132, 6.98132), (-2.18166, 2.0944), (-6.98132, 6.98132)]

# Grandeurs dans le plan du bras (x radial, z vertical)
_U0 = J2_OFF[0]
_V0 = Z_BASE + J2_OFF[2]
_A = (J3_OFF[0], J3_OFF[2])                       # J2 -> J3
_B = (J4_OFF[0] + J5_OFF[0], J4_OFF[2] + J5_OFF[2])  # J3 -> centre poignet (J5)
_LA2 = _A[0] ** 2 + _A[1] ** 2
_LB2 = _B[0] ** 2 + _B[1] ** 2
_KA = _A[0] * _B[0] + _A[1] * _B[1]
_KB = _A[0] * _B[1] - _A[1] * _B[0]
REACH_MAX = math.sqrt(_LA2) + math.sqrt(_LB2)


def _wrap_near(a, ref):
    while a - ref > math.pi:
        a -= 2 * math.pi
    while a - ref < -math.pi:
        a += 2 * math.pi
    return a


def ik_down(x, y, z, yaw, tool_len, q_ref=None):
    """Position TCP (repère base robot), outil vertical vers le bas, lacet `yaw`
    = direction de l'axe Y local du préhenseur. Retourne [q1..q6] ou None."""
    if q_ref is None:
        q_ref = [0.0] * 6
    wz = z + tool_len + FLANGE + J6_OFF[0]
    q1 = math.atan2(y, x)
    u = math.hypot(x, y) - _U0
    v = wz - _V0
    d2 = u * u + v * v
    k = (d2 - _LA2 - _LB2) / 2.0
    r = math.hypot(_KA, _KB)
    if abs(k) > r:
        return None
    phi = math.atan2(_KB, _KA)
    best = None
    for sgn in (1, -1):
        q3 = phi + sgn * math.acos(k / r)
        q3 = _wrap_near(q3, 0.0)
        c, s = math.cos(q3), math.sin(q3)
        wx = _A[0] + _B[0] * c + _B[1] * s
        wzp = _A[1] - _B[0] * s + _B[1] * c
        q2 = math.atan2(wzp, wx) - math.atan2(v, u)
        q2 = _wrap_near(q2, 0.0)
        q5 = math.pi / 2 - q2 - q3
        q5 = _wrap_near(q5, 0.0)
        q6 = _wrap_near(q1 - yaw, q_ref[5])
        q = [q1, q2, q3, 0.0, q5, q6]
        if all(lo - 1e-6 <= qi <= hi + 1e-6 for qi, (lo, hi) in zip(q, LIMITS)):
            cost = abs(q3 - q_ref[2]) + abs(q2 - q_ref[1])
            if best is None or cost < best[0]:
                best = (cost, q)
    return None if best is None else best[1]


# ---------------------------------------------------------------- FK (contrôle)
def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _tr(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def _rot(axis, a):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0],
            [0, 0, 0, 1]]


def fk(q, tool_len):
    """Pose 4x4 du TCP (repère préhenseur : z vers l'outil)."""
    T = _tr(0, 0, Z_BASE)
    T = _mat_mul(T, _rot((0, 0, 1), q[0]))
    T = _mat_mul(T, _tr(*J2_OFF)); T = _mat_mul(T, _rot((0, 1, 0), q[1]))
    T = _mat_mul(T, _tr(*J3_OFF)); T = _mat_mul(T, _rot((0, 1, 0), q[2]))
    T = _mat_mul(T, _tr(*J4_OFF)); T = _mat_mul(T, _rot((1, 0, 0), q[3]))
    T = _mat_mul(T, _tr(*J5_OFF)); T = _mat_mul(T, _rot((0, 1, 0), q[4]))
    T = _mat_mul(T, _tr(*J6_OFF)); T = _mat_mul(T, _rot((1, 0, 0), q[5]))
    T = _mat_mul(T, _tr(FLANGE, 0, 0)); T = _mat_mul(T, _rot((1, 1, 1), 2 * math.pi / 3))
    T = _mat_mul(T, _tr(0, 0, tool_len))
    return T


if __name__ == "__main__":
    L = 0.30
    for tgt in [(0, -1.7, 1.05, 0), (1.8, 0.3, 0.56, 0), (1.8, -0.3, 1.61, 0), (0.0, -1.7, 1.45, 0.0)]:
        q = ik_down(*tgt, L)
        T = fk(q, L)
        yaw_y = math.atan2(T[1][1], T[0][1])
        print(tgt, [round(a, 3) for a in q], "->", [round(T[i][3], 4) for i in range(3)],
              "zaxis", [round(T[i][2], 3) for i in range(3)], "yaw", round(yaw_y, 3))
