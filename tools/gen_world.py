#!/usr/bin/env python3
"""Génère worlds/palettisation.wbt : cellule de palettisation robotisée dans un hangar logistique.

Implantation (m, repère ENU, z vers le haut) :
  - Robot ABB IRB 4600-40 en (0, 0), préhenseur à 12 ventouses à soufflets
  - Fermeuse de cartons + convoyeur à bande d'entrée le long de x (y = -1.70, bande à 0.75 m)
  - Sas de formation de lots x ∈ [-0.45, 0.40] (butée pneumatique escamotable + butée fixe)
  - Ligne palettes = convoyeurs à chaînes bas (3 brins, dessus à 0.10 m, prise chariot entre brins), x = 1.80 :
        P0 magasin palettes vides y=+2.4 | P1 palettisation y=0 | P2 banderoleuse à plateau tournant y=-2.8
        P3 poste d'évacuation (hors carter) y=-5.4
  - Carter périmétrique profilés alu + polycarbonate x ∈ [-2.8, 3.8], y ∈ [-4.2, 3.4]
  - Zone de stockage (longrines + racks) dans le même hangar, x ∈ [7, 18], y ∈ [-17, -11]
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "worlds", "palettisation.wbt")

R = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"
PROTOS = [
    "objects/backgrounds/protos/TexturedBackground.proto",
    "appearances/protos/RoughConcrete.proto",
    "appearances/protos/CorrugatedMetal.proto",
    "appearances/protos/GalvanizedMetal.proto",
    "appearances/protos/Rubber.proto",
    "appearances/protos/Cardboard.proto",
    "robots/abb/irb/protos/Irb4600-40.proto",
    "objects/factory/pallet/protos/WoodenPallet.proto",
    "objects/factory/pallet/protos/WoodenPalletStack.proto",
    "objects/factory/containers/protos/CardboardBox.proto",
    "objects/factory/forklift/protos/Forklift.proto",
    "objects/factory/fire_extinguisher/protos/FireExtinguisher.proto",
    "humans/pedestrian/protos/Pedestrian.proto",
]
IMPORTABLE = ("WoodenPallet.proto", "CardboardBox.proto", "Pedestrian.proto")

# --------------------------------------------------------------- constantes partagées avec cell_plc.py
CONV_Y, CONV_Z, CONV_W = -1.70, 0.75, 0.70
PAL_X, PAL_Z = 1.80, 0.10
ST = {"P0": 2.4, "P1": 0.0, "P2": -2.8, "P3": -5.4}
FENCE = (-2.8, 3.8, -4.2, 3.4)
LC_X0, LC_X1 = 1.22, 2.38
LC_Z = [0.30 + 0.20 * i for i in range(9)]
WRAP_MAST_X = PAL_X + 1.35
SPOTS = [8.0 + 1.15 * i for i in range(8)]
SPOT_Y = -12.6
BX0, BX1, BY0, BY1, BH = -12.0, 19.0, -18.0, 8.5, 8.0   # enveloppe du hangar
YELLOW = "1 0.78 0.05"
BLACK = "0.08 0.08 0.08"

out = []


def w(s):
    out.append(s)


def f(v):
    return " ".join("%.4g" % x for x in v)


def app(color, rough=0.5, metal=0.0, emissive=None, transp=0.0, def_=None):
    d = "DEF %s " % def_ if def_ else ""
    e = " emissiveColor %s" % emissive if emissive else ""
    t = " transparency %g" % transp if transp else ""
    return "%sPBRAppearance { baseColor %s roughness %g metalness %g%s%s }" % (d, color, rough, metal, e, t)


class Prim(str):
    """Primitive (box / cylindre) : se comporte comme sa chaîne VRML, mais peut être fusionnée par render()."""


def _rotm(rot):
    if not rot:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z, a = rot
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s_, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return ((c + x * x * C, x * y * C - z * s_, x * z * C + y * s_),
            (y * x * C + z * s_, c + y * y * C, y * z * C - x * s_),
            (z * x * C - y * s_, z * y * C + x * s_, c + z * z * C))


def _mergeable(appearance, def_):
    return def_ is None and appearance.startswith("PBRAppearance") and "DEF " not in appearance


def box(size, appearance, pos=(0, 0, 0), rot=None, def_=None, cs=True):
    r = " rotation %s" % f(rot) if rot else ""
    d = "DEF %s " % def_ if def_ else ""
    c = "" if cs else " castShadows FALSE"
    o = Prim("Pose { translation %s%s children [ Shape {%s appearance %s geometry %sBox { size %s } } ] }" % (
        f(pos), r, c, appearance, d, f(size)))
    o.geo = ("box", tuple(size), tuple(pos), rot, appearance, cs) if _mergeable(appearance, def_) else None
    return o


def cyl(radius, height, appearance, pos=(0, 0, 0), rot=None):
    r = " rotation %s" % f(rot) if rot else ""
    o = Prim("Pose { translation %s%s children [ Shape { appearance %s geometry Cylinder { radius %g height %g "
             "subdivision 20 } } ] }" % (f(pos), r, appearance, radius, height))
    o.geo = ("cyl", (radius, height), tuple(pos), rot, appearance, True) if _mergeable(appearance, None) else None
    return o


def _flatten(items):
    for it in items:
        if isinstance(it, (list, tuple)):
            yield from _flatten(it)
        elif it:
            yield it


def render(items):
    """Fusionne toutes les primitives de même matériau en un IndexedFaceSet (1 appel de rendu par matériau)."""
    groups, others = {}, []
    for it in _flatten(items):
        g = getattr(it, "geo", None)
        if g is None:
            others.append(str(it))
            continue
        kind, dims, pos, rot, appearance, cs = g
        pts, idx = groups.setdefault((appearance, cs), ([], []))
        Rm = _rotm(rot)

        def tr(v):
            return tuple(pos[i] + Rm[i][0] * v[0] + Rm[i][1] * v[1] + Rm[i][2] * v[2] for i in range(3))
        b = len(pts)
        if kind == "box":
            sx, sy, sz = dims
            for i in range(8):
                pts.append(tr(((sx if i & 4 else -sx) / 2, (sy if i & 2 else -sy) / 2, (sz if i & 1 else -sz) / 2)))
            for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
                idx.append([k + b for k in q])
        else:
            r_, h = dims
            N = 14
            for i in range(N):
                a = 2 * math.pi * i / N
                pts.append(tr((r_ * math.cos(a), r_ * math.sin(a), -h / 2)))
                pts.append(tr((r_ * math.cos(a), r_ * math.sin(a), h / 2)))
            for i in range(N):
                j = (i + 1) % N
                idx.append([b + 2 * i, b + 2 * j, b + 2 * j + 1, b + 2 * i + 1])
            idx.append([b + 2 * i + 1 for i in range(N)])
            idx.append([b + 2 * i for i in reversed(range(N))])
    out_ = list(others)
    for (appearance, cs), (pts, idx) in groups.items():
        out_.append("Shape {%s appearance %s geometry IndexedFaceSet { coord Coordinate { point [ %s ] } "
                    "coordIndex [ %s ] creaseAngle 0.6 } }" % (
                        "" if cs else " castShadows FALSE", appearance,
                        ", ".join(f(p_) for p_ in pts), " ".join(" ".join(map(str, q)) + " -1" for q in idx)))
    return " ".join(out_)


def cyl_x(radius, length, appearance, pos):
    return cyl(radius, length, appearance, pos, (0, 1, 0, math.pi / 2))


def cyl_y(radius, length, appearance, pos):
    return cyl(radius, length, appearance, pos, (1, 0, 0, math.pi / 2))


def solid(name, pos, children, rot=None, def_=None, bo=None):
    d = "DEF %s " % def_ if def_ else ""
    r = " rotation %s" % f(rot) if rot else ""
    b = " boundingObject %s" % bo if bo else ""
    return '%sSolid { translation %s%s name "%s" children [ %s ]%s }' % (d, f(pos), r, name, render(children), b)


ALU = app("0.66 0.67 0.66", 0.35, 0.85)
SLOT = app("0.25 0.26 0.27", 0.6, 0.5)
STEEL = app("0.55 0.57 0.6", 0.35, 0.85)
GALVA = "GalvanizedMetal { }"
DARK = app("0.2 0.2 0.22", 0.6, 0.3)
RAL5010 = app("0.06 0.24 0.45", 0.45, 0.3)        # bleu gentiane (châssis convoyeurs)
RAL7035 = app("0.80 0.81 0.79", 0.5, 0.2)         # gris clair (machines)
RAL2004 = app("0.95 0.38 0.05", 0.45, 0.2)        # orange (lisses racks)
MOTOR = app("0.30 0.38 0.48", 0.45, 0.5)
YEL = app(YELLOW, 0.5, 0.1)
BLK = app(BLACK, 0.6, 0.1)
RED = app("0.85 0.05 0.05", 0.4, 0.1)
BLUE = app("0.1 0.3 0.85", 0.4, 0.1)
PC = app("0.72 0.77 0.8", 0.08, 0, transp=0.8)


def profile(size, pos):
    """Profilé alu rainuré (barre de section carrée), orienté selon sa plus grande dimension."""
    parts = [box(size, ALU, pos)]
    i = max(range(3), key=lambda k: size[k])
    for k in range(3):
        if k == i:
            continue
        for sgn in (-1, 1):
            s = list(size)
            s[k] = 0.002
            o = [3 - k - i]  # l'autre axe
            s[o[0]] = 0.01
            p = list(pos)
            p[k] += sgn * size[k] / 2
            parts.append(box(s, SLOT, p, cs=False))
    return parts


# =============================================================== en-tête, ambiance intérieure
w("#VRML_SIM R2025a utf8")
for p in PROTOS:
    w('%sEXTERNPROTO "%s%s"' % ("IMPORTABLE " if p.endswith(IMPORTABLE) else "", R, p))
w('EXTERNPROTO "../protos/GuardBay.proto"')
w("""WorldInfo {
  info [ "Cellule de palettisation robotisée dans un hangar logistique - ABB IRB 4600-40 + préhenseur à ventouses"
         "Touches (vue 3D active) : E = arrêt d'urgence, D = porte d'accès, R = réarmement, P = piéton, S = arrêt fin de cycle"
       ]
  title "Cellule de palettisation"
  basicTimeStep 16
}
Viewpoint {
  orientation 0.16952 0.44512 -0.87928 0.81748
  position -9.5 6.8 6.2
  near 0.1
}
TexturedBackground { texture "factory" skybox FALSE skyColor [ 0.17 0.18 0.2 ] luminosity 0.9 }
DirectionalLight { direction -0.35 -0.25 -1 intensity 1.9 castShadows TRUE }
DirectionalLight { direction 0.4 0.5 -1 intensity 0.7 castShadows FALSE }
""")

# =============================================================== bâtiment (intérieur du hangar)
W_, D_ = BX1 - BX0, BY1 - BY0
cxb, cyb = (BX0 + BX1) / 2, (BY0 + BY1) / 2
w('DEF SOL Solid { translation %s name "dallage beton" children [ %s ] boundingObject Pose { translation 0 0 -0.05 '
  'children [ Box { size %g %g 0.1 } ] } locked TRUE }' % (
      f((cxb, cyb, 0)), box((W_, D_, 0.1), 'RoughConcrete { colorOverride 0.82 0.83 0.85 textureTransform '
                             'TextureTransform { scale 14 12 } }', (0, 0, -0.05)), W_, D_))
bld = []
clad = 'CorrugatedMetal { colorOverride 0.78 0.8 0.83 textureTransform TextureTransform { scale %g 3 } }'
socle = app("0.62 0.62 0.6", 0.9, 0)
for (p, s) in (((cxb, BY0, 0), (W_, 0.2, 0)), ((cxb, BY1, 0), (W_, 0.2, 0)),
               ((BX0, cyb, 0), (0.2, D_, 0)), ((BX1, cyb, 0), (0.2, D_, 0))):
    L = max(s[0], s[1])
    bld.append(box((s[0] or 0.2, s[1] or 0.2, 1.5), socle, (p[0], p[1], 0.75)))                 # soubassement béton
    bld.append(box((s[0] or 0.2, s[1] or 0.2, BH - 1.5), clad % (L / 2), (p[0], p[1], 1.5 + (BH - 1.5) / 2)))
    bld.append(box((s[0] + 0.01 if s[0] else 0.21, s[1] + 0.01 if s[1] else 0.21, 0.6),
                   app("0.75 0.85 0.95", 0.1, 0, emissive="0.35 0.4 0.45", transp=0.15),
                   (p[0], p[1], BH - 1.0)))                                                       # bandeau éclairant
# poteaux de charpente + protections pied de poteau
COL = app("0.28 0.35 0.45", 0.5, 0.4)
xs = [BX0 + 0.3 + i * 6.2 for i in range(6)]
for x in xs:
    for y in (BY0 + 0.3, BY1 - 0.3):
        bld.append(box((0.3, 0.3, BH), COL, (x, y, BH / 2)))
        bld.append(box((0.4, 0.4, 1.0), YEL, (x, y, 0.5)))
        bld.append(box((0.41, 0.41, 0.15), BLK, (x, y, 0.3)))
        bld.append(box((0.41, 0.41, 0.15), BLK, (x, y, 0.75)))
# fermes de toiture + pannes + luminaires suspendus (toiture non représentée : on voit la sous-face sombre)
for x in xs:
    bld.append(box((0.25, D_, 0.35), COL, (x, cyb, BH + 0.2)))
    bld.append(box((0.15, D_, 0.2), COL, (x, cyb, BH + 1.4)))
    for k in range(9):
        yk = BY0 + k * D_ / 8
        bld.append(box((0.08, 0.08, 1.2), COL, (x, yk, BH + 0.8)))
for k in range(7):
    yk = BY0 + 0.5 + k * (D_ - 1) / 6
    bld.append(box((W_, 0.12, 0.18), COL, (cxb, yk, BH + 1.55)))
lamp = app("1 0.98 0.92", 0.3, 0, emissive="1 0.97 0.9")
for x in [BX0 + 3.4 + 6.2 * i for i in range(5)]:
    for y in [BY0 + 3 + 6.5 * j for j in range(4)]:
        bld.append(cyl(0.008, 1.0, DARK, (x, y, BH - 0.5)))
        bld.append(cyl(0.28, 0.18, DARK, (x, y, BH - 1.05)))
        bld.append(cyl(0.25, 0.02, lamp, (x, y, BH - 1.15)))
# portes sectionnelles de quai (mur nord) + butoirs
for x in (4.0, 8.5, 13.0):
    bld.append(box((3.2, 0.06, 3.6), app("0.75 0.76 0.78", 0.4, 0.5), (x, BY1 - 0.12, 1.8)))
    for k in range(1, 8):
        bld.append(box((3.2, 0.065, 0.02), DARK, (x, BY1 - 0.12, 0.45 * k)))
    bld.append(box((3.5, 0.1, 0.15), YEL, (x, BY1 - 0.16, 3.7)))
    for sx in (-1.75, 1.75):
        bld.append(box((0.16, 0.1, 3.7), YEL, (x + sx, BY1 - 0.16, 1.85)))
        bld.append(box((0.2, 0.25, 0.3), BLK, (x + sx * 0.8, BY1 - 0.25, 1.1)))
w(solid("batiment hangar", (0, 0, 0), bld))

# =============================================================== marquages au sol
marks = []


def floor_line(p0, p1, width, color):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    marks.append(box((L, width, 0.004), app(color, 0.7), ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, 0.002),
                     (0, 0, 1, math.atan2(dy, dx)), cs=False))


def floor_rect(x0, x1, y0, y1, width, color):
    floor_line((x0, y0), (x1, y0), width, color)
    floor_line((x1, y0), (x1, y1), width, color)
    floor_line((x1, y1), (x0, y1), width, color)
    floor_line((x0, y1), (x0, y0), width, color)


floor_rect(FENCE[0] - 0.35, FENCE[1] + 0.35, FENCE[2] - 0.35, FENCE[3] + 0.35, 0.1, YELLOW)
floor_rect(FENCE[0] - 0.5, FENCE[1] + 0.5, FENCE[2] - 0.5, FENCE[3] + 0.5, 0.05, BLACK)
floor_rect(1.1, 2.5, -6.25, -4.45, 0.08, "1 0.45 0")                                  # zone de reprise chariot
for y in (-8.3, -10.7):
    floor_line((0.6, y), (18.0, y), 0.1, "0.1 0.3 0.85")                             # allée chariots
for (a, b) in (((-5.0, 6.5), (-5.0, -7.5)), ((-4.2, 6.5), (-4.2, -1.0)), ((-5.0, -7.5), (0.6, -7.5)),
               ((-4.2, 6.5), (6.0, 6.5)), ((-5.0, 7.3), (6.0, 7.3))):
    floor_line(a, b, 0.1, "0.1 0.65 0.2")                                             # allées piétons
w('Solid { name "marquage sol" children [ %s ] locked TRUE }' % render(marks))

# =============================================================== robot + préhenseur à ventouses
G = [
    # bride ISO 9409 + plaque d'adaptation + changeur d'outil
    "Pose { rotation 1 0 0 1.5708 children [ %s ] }" % cyl(0.1, 0.02, ALU, (0, 0.01, 0)),
    box((0.17, 0.17, 0.05), DARK, (0, 0, 0.045)),
    box((0.2, 0.2, 0.012), ALU, (0, 0, 0.076)),
    # poutre transversale (le long de x) en profilé 45x90, sous la plaque
    profile((0.56, 0.09, 0.045), (0, 0, 0.1045)),
]
for bx in (-0.22, 0.0, 0.22):                      # 3 poutres porte-ventouses (le long de y)
    G.append(profile((0.045, 0.86, 0.045), (bx, 0, 0.1495)))
    G.append(box((0.04, 0.04, 0.004), BLK, (bx, 0.432, 0.1495), cs=False))
    G.append(box((0.04, 0.04, 0.004), BLK, (bx, -0.432, 0.1495), cs=False))
    G.append(cyl_y(0.005, 0.72, app("0.15 0.4 0.85", 0.4, 0), (bx + 0.03, 0, 0.16)))          # tuyau PU bleu
# équerres d'assemblage
for bx in (-0.22, 0.22):
    for by in (-0.05, 0.05):
        G.append(box((0.035, 0.004, 0.035), ALU, (bx, by, 0.13)))
# générateur de vide (éjecteur) + silencieux + vacuostat + bloc de distribution
G.append(box((0.16, 0.08, 0.06), app("0.1 0.3 0.7", 0.4, 0.2), (0.11, 0.2, 0.05)))
G.append(cyl_x(0.018, 0.08, BLK, (0.23, 0.2, 0.05)))
G.append(box((0.03, 0.03, 0.03), BLK, (0.07, 0.25, 0.015)))
G.append(cyl(0.008, 0.02, app("0.1 0.9 0.2", emissive="0 0.6 0"), (0.07, 0.25, -0.005)))
G.append(box((0.5, 0.05, 0.03), ALU, (0, 0.2, 0.105)))
G.append(cyl(0.006, 0.09, app("0.15 0.4 0.85", 0.4, 0), (0.0, 0.2, 0.05)))
# 12 ventouses à soufflet 1,5 plis sur compensateurs de niveau
BELLOW = app("0.12 0.12 0.13", 0.75, 0)
LIP = app("0.05 0.15 0.45", 0.7, 0)
for bx in (-0.22, 0.0, 0.22):
    for by in (-0.3, -0.1, 0.1, 0.3):
        G.append(cyl(0.012, 0.045, STEEL, (bx, by, 0.194)))        # corps compensateur
        G.append(cyl(0.016, 0.006, STEEL, (bx, by, 0.175)))        # écrou
        G.append(cyl(0.007, 0.03, STEEL, (bx, by, 0.229)))         # tige
        G.append(cyl(0.013, 0.012, ALU, (bx, by, 0.25)))           # raccord
        G.append(cyl(0.024, 0.009, BELLOW, (bx, by, 0.2605)))      # soufflet pli 1
        G.append(cyl(0.031, 0.008, BELLOW, (bx, by, 0.269)))
        G.append(cyl(0.024, 0.006, BELLOW, (bx, by, 0.276)))       # pli 2
        G.append(cyl(0.037, 0.016, LIP, (bx, by, 0.287)))          # lèvre
        G.append(cyl(0.004, 0.03, app("0.15 0.4 0.85", 0.4, 0), (bx + 0.03, by, 0.19)))  # descente tuyau
gripper = """DEF GRIPPER Pose {
  children [
    %s
    DEF TCP Pose { translation 0 0 0.30 }
    DistanceSensor {
      translation 0.11 0.2 0.26 rotation 0 1 0 -1.5708 name "capteur_presence_colis"
      lookupTable [ 0 0 0, 0.25 0.25 0 ] type "generic"
      children [ %s ]
    }
    Emitter { name "robot_tx" channel 2 }
    Receiver { name "plc_rx" channel 1 }
  ]
}""" % (render(G), box((0.02, 0.02, 0.03), YEL))
w("""DEF IRB Irb4600-40 {
  translation 0 0 0.20
  name "IRB 4600-40 palettiseur"
  controller "palletizer_robot"
  supervisor TRUE
  staticBase TRUE
  color 1 0.45 0
  handSlot [ %s ]
}""" % gripper)
ped = [box((0.9, 0.9, 0.18), app("0.3 0.32 0.35", 0.5, 0.6), (0, 0, 0.09)),
       box((1.0, 1.0, 0.02), app("0.3 0.32 0.35", 0.5, 0.6), (0, 0, 0.01))]
for sx in (-0.42, 0.42):
    for sy in (-0.42, 0.42):
        ped.append(cyl(0.02, 0.05, STEEL, (sx, sy, 0.03)))
w(solid("embase robot", (0, 0, 0), ped, bo="Pose { translation 0 0 0.1 children [ Box { size 1 1 0.2 } ] }"))

# =============================================================== convoyeurs à bande (colis)
def belt_conveyor(name, tt_def, x0, x1, y, ztop, width, drive_at_x1=True, color="0.30 0.36 0.32"):
    L = x1 - x0
    cx = (x0 + x1) / 2
    p = []
    belt = ('Rubber { type "dotted" colorOverride %s textureTransform DEF %s TextureTransform { scale %g %g } }'
            % (color, tt_def, L * 3, width * 3))
    p.append(box((L, width, 0.012), belt, (0, 0, ztop - 0.006)))
    p.append(box((L, width - 0.02, 0.01), app("0.15 0.16 0.15", 0.8), (0, 0, ztop - 0.1)))
    for ex in (-L / 2, L / 2):
        p.append(cyl_y(0.048, width, STEEL, (ex, 0, ztop - 0.054)))
        p.append(cyl_y(0.02, width + 0.12, STEEL, (ex, 0, ztop - 0.054)))
    yb = width / 2 + 0.025
    for sy in (-1, 1):
        p.append(profile((L + 0.08, 0.04, 0.12), (0, sy * yb, ztop - 0.065)))
    nleg = max(2, int(math.ceil(L / 1.6)) + 1)
    for i in range(nleg):
        lx = -L / 2 + 0.1 + i * (L - 0.2) / (nleg - 1)
        for sy in (-1, 1):
            p.append(profile((0.045, 0.045, ztop - 0.125), (lx, sy * yb, (ztop - 0.125) / 2 + 0.01)))
            p.append(cyl(0.035, 0.015, BLK, (lx, sy * yb, 0.0075)))
        p.append(profile((0.045, width + 0.05, 0.045), (lx, 0, 0.25)))
    # guides latéraux inox réglables
    ng = max(2, int(L / 1.0) + 1)
    for sy in (-1, 1):
        for zg in (ztop + 0.07, ztop + 0.2):
            p.append(cyl_x(0.009, L - 0.02, STEEL, (0, sy * 0.325, zg)))
        for i in range(ng):
            gx = -L / 2 + 0.05 + i * (L - 0.1) / (ng - 1)
            p.append(box((0.02, 0.02, 0.26), ALU, (gx, sy * (yb + 0.02), ztop + 0.08)))
            for zg in (ztop + 0.07, ztop + 0.2):
                p.append(box((0.015, 0.045, 0.015), ALU, (gx, sy * (yb - 0.005), zg)))
    # motoréducteur
    dx = L / 2 - 0.06 if drive_at_x1 else -L / 2 + 0.06
    p.append(box((0.15, 0.13, 0.15), MOTOR, (dx, -(yb + 0.1), ztop - 0.06)))
    p.append(cyl_y(0.065, 0.24, MOTOR, (dx, -(yb + 0.29), ztop - 0.06)))
    p.append(cyl_y(0.07, 0.03, DARK, (dx, -(yb + 0.42), ztop - 0.06)))
    p.append(box((0.08, 0.06, 0.06), DARK, (dx, -(yb + 0.27), ztop + 0.03)))
    w(solid(name, (cx, y, 0), p, bo="Pose { translation 0 0 %g children [ Box { size %g %g 0.12 } ] }"
            % (ztop - 0.06, L, width)))


belt_conveyor("convoyeur a bande entree colis", "TT_C1", -8.0, -0.47, CONV_Y, CONV_Z, CONV_W)
belt_conveyor("convoyeur a bande sas formation lots", "TT_C2", -0.43, 0.55, CONV_Y, CONV_Z, CONV_W,
              color="0.25 0.3 0.4")

# fermeuse de cartons en amont (machine de fin de ligne de production)
fm = [box((1.7, 1.1, 0.75), RAL7035, (0, 0, 0.375)),
      box((1.7, 0.12, 0.95), RAL7035, (0, 0.49, 1.22)), box((1.7, 0.12, 0.95), RAL7035, (0, -0.49, 1.22)),
      box((1.7, 1.1, 0.25), RAL7035, (0, 0, 1.82)),
      box((1.6, 0.86, 0.85), PC, (0, 0, 1.22)),
      box((1.72, 1.12, 0.05), app("0.95 0.38 0.05", 0.4, 0.2), (0, 0, 1.95)),
      box((0.35, 0.25, 0.4), RAL7035, (-0.5, -0.68, 1.3)),
      box((0.25, 0.01, 0.18), app("0.1 0.2 0.3", 0.2, 0, emissive="0.1 0.3 0.5"), (-0.5, -0.81, 1.35)),
      cyl(0.18, 0.06, app("0.5 0.35 0.2", 0.7), (0.2, 0, 1.75), (1, 0, 0, 1.5708)),
      box((1.7, 0.7, 0.012), 'Rubber { type "dotted" colorOverride 0.3 0.36 0.32 }', (0, 0, CONV_Z - 0.006))]
w(solid("fermeuse de cartons (amont)", (-8.85, CONV_Y, 0), fm))

# butée d'entrée de sas (vérin pneumatique escamotable) + butée fixe de fin
w(solid("butee sas (verin)", (-0.45, CONV_Y, 0), [
    cyl(0.03, 0.35, STEEL, (0, 0, 0.45)), box((0.08, 0.12, 0.25), ALU, (0, 0, 0.42)),
    box((0.02, 0.02, 0.02), BLUE, (0.05, 0.05, 0.5)),
    'DEF BUTEE_SAS Pose { translation 0 0 0.72 children [ %s %s ] }' % (
        box((0.025, 0.62, 0.22), YEL), box((0.01, 0.6, 0.18), BLK, (0.015, 0, 0.0)))]))
w(solid("butee fixe fin de sas", (0.42, CONV_Y, CONV_Z + 0.12), [
    box((0.03, 0.74, 0.24), ALU), box((0.02, 0.6, 0.12), BLK, (-0.02, 0, 0.0))],
    bo="Box { size 0.03 0.74 0.24 }"))
# cellules photo-électriques (visuels) : émetteur côté robot, réflecteur en face
for x in (-7.5, -3.0, -0.58, -0.25, 0.25):
    w(solid("cellule %.2f" % x, (x, CONV_Y, 0.9), [
        box((0.03, 0.04, 0.05), app("0.95 0.6 0.1"), (0, 0.42, 0)), box((0.012, 0.01, 0.012), BLK, (0, 0.40, 0.01)),
        box((0.04, 0.005, 0.04), app("0.9 0.1 0.1", 0.2), (0, -0.4, 0))]))

# =============================================================== ligne palettes : convoyeurs à chaînes bas
def chain_conveyor(name, y0, y1, drive_end=None, parent_pose=None, base_z=0.0):
    """3 brins de chaîne sous les semelles de la palette (x = ±0.36, 0) ; dessus des chaînes à 0.10 m."""
    L = y1 - y0
    p = []
    top = PAL_Z - base_z
    for dx in (-0.36, 0.0, 0.36):
        p.append(box((0.07, L, top - 0.025), RAL5010, (dx, 0, (top - 0.025) / 2)))
        p.append(box((0.03, L - 0.04, 0.025), 'GalvanizedMetal { textureTransform TextureTransform { scale 1 %g } }'
                     % (L * 10), (dx, 0, top - 0.0125)))
        p.append(box((0.074, L, 0.006), app("0.2 0.2 0.2", 0.7), (dx, 0, top - 0.025)))
    for ty in (-L / 2 + 0.06, 0, L / 2 - 0.06):
        p.append(box((0.9, 0.06, 0.04), RAL5010, (0, ty, 0.02)))
    if drive_end is not None:
        ty = (L / 2 - 0.08) * drive_end
        p.append(cyl_x(0.02, 0.92, STEEL, (0, ty, top - 0.03)))
        for dx in (-0.36, 0.0, 0.36):
            p.append(box((0.1, 0.16, 0.11), YEL, (dx, ty, top - 0.045)))               # carters de pignons
        p.append(box((0.14, 0.16, 0.14), RAL5010, (0.53, ty, 0.075)))
        p.append(cyl_x(0.06, 0.22, MOTOR, (0.71, ty, 0.075)))
        p.append(cyl_x(0.065, 0.02, DARK, (0.83, ty, 0.075)))
    return p


for nm, a, b, d in (("convoyeur chaines magasin P0", 1.72, 3.1, -1), ("convoyeur chaines palettisation P1", -0.66, 1.66, -1),
                    ("convoyeur chaines transfert", -2.06, -0.72, 1), ("convoyeur chaines evacuation P3", -6.15, -3.5, 1)):
    w(solid(nm, (PAL_X, (a + b) / 2, 0), chain_conveyor(nm, a, b, d)))

# magasin de palettes vides (dépileur) : 4 colonnes + griffes pneumatiques
w('DEF MAGASIN WoodenPalletStack { translation %s rotation 0 0 1 1.5708 name "magasin palettes vides" '
  'palletNumber 10 lateralMisalignment 0.005 longitudinalMisalignment 0.005 }' % f((PAL_X, ST["P0"], PAL_Z + 0.144)))
mg = []
for sx in (-0.5, 0.5):
    for sy in (-0.68, 0.68):
        mg.append(profile((0.06, 0.06, 2.1), (sx, sy, 1.05)))
    mg.append(profile((0.06, 1.42, 0.06), (sx, 0, 2.1)))
    mg.append(box((0.08, 0.5, 0.06), YEL, (sx * 0.92, 0, 0.32)))                       # griffes
    mg.append(cyl_x(0.03, 0.18, ALU, (sx * 1.08, 0, 0.32)))                           # vérins
for sy in (-0.68, 0.68):
    mg.append(profile((1.06, 0.06, 0.06), (0, sy, 2.1)))
w(solid("depileur palettes vides", (PAL_X, ST["P0"], 0), mg))

# =============================================================== banderoleuse automatique à plateau tournant
BX, BY = PAL_X, ST["P2"]
plat = [cyl(1.0, 0.02, app("0.35 0.36 0.38", 0.5, 0.5), (0, 0, 0.01))]
plat += chain_conveyor("plateau", -0.65, 0.65, None, base_z=0.045)
plat = ["Pose { translation 0 0 0.02 children [ %s ] }" % render(plat)]
wr = [cyl(1.08, 0.025, YEL, (0, 0, 0.0125)), cyl(1.04, 0.03, app("0.2 0.2 0.22", 0.6, 0.4), (0, 0, 0.03)),
      'DEF PLATEAU Pose { translation 0 0 0.025 children [ %s ] }' % render(plat)]
mx = WRAP_MAST_X - BX
wr += [box((0.6, 0.5, 0.02), RAL7035, (mx, 0, 0.01)),
       box((0.22, 0.22, 2.55), RAL7035, (mx, 0, 1.295)),
       box((0.24, 0.24, 0.05), DARK, (mx, 0, 2.595)),
       box((0.02, 0.06, 2.4), DARK, (mx - 0.115, 0, 1.3)),                             # crémaillère / guide
       box((0.3, 0.12, 0.3), RAL7035, (mx, 0, 2.48)),                                   # moteur de levée
       box((0.9, 0.4, 0.22), app("0.2 0.2 0.22", 0.6, 0.4), (mx - 0.35, 0, 0.11)),       # pied/caisson
       # armoire électrique + pupitre
       box((0.5, 0.35, 1.4), RAL7035, (mx + 0.05, 0.6, 0.7)),
       box((0.3, 0.01, 0.2), app("0.1 0.2 0.3", 0.2, 0, emissive="0.1 0.3 0.5"), (mx + 0.05, 0.42, 1.15)),
       cyl(0.03, 0.03, RED, (mx - 0.08, 0.425, 0.9), (1, 0, 0, 1.5708)),
       """DEF CHARIOT_FILM Pose { translation %g 0 0.3 children [
            %s %s %s %s %s %s
          ] }""" % (mx - 0.2,
                    box((0.16, 0.42, 0.5), RAL7035, (0.05, 0, 0)),
                    cyl(0.09, 0.5, app("0.85 0.92 0.98", 0.15, 0, transp=0.25), (-0.12, 0.08, 0)),
                    cyl(0.04, 0.52, DARK, (-0.12, 0.08, 0)),
                    cyl(0.035, 0.5, STEEL, (-0.05, -0.1, 0)),
                    cyl(0.035, 0.5, STEEL, (-0.12, -0.15, 0)),
                    box((0.14, 0.2, 0.14), MOTOR, (0.18, 0, 0.12)))]
w(solid("banderoleuse plateau tournant", (BX, BY, 0), wr))

# =============================================================== carter de protection (profilés alu + polycarbonate)
def fence_side(p0, p1, segments, h=2.2):
    """segments : liste (s0, s1, type) ; type = panel | open | tunnel | door"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    yaw = math.atan2(dy, dx)
    for s0, s1, kind in segments:
        if kind == "open" or kind == "door":
            continue
        if kind == "tunnel":
            w('GuardBay { translation %s rotation 0 0 1 %g name "travee %g %g" width %g height %g openZ0 0 openZ1 1.45 '
              'rightPost TRUE }' % (f((p0[0] + ux * s0, p0[1] + uy * s0, 0)), yaw, p0[0] + ux * s0, p0[1] + uy * s0,
                                    s1 - s0, h))
            continue
        n = max(1, math.ceil((s1 - s0) / 1.4 - 1e-6))
        bw = (s1 - s0) / n
        for k in range(n):
            s = s0 + k * bw
            w('GuardBay { translation %s rotation 0 0 1 %g name "travee %g %g" width %g height %g%s }' % (
                f((p0[0] + ux * s, p0[1] + uy * s, 0)), yaw, round(p0[0] + ux * s, 3), round(p0[1] + uy * s, 3),
                bw, h, " rightPost TRUE" if k == n - 1 else ""))


x0, x1, yb, yt = FENCE
Lx, Ly = x1 - x0, yt - yb
lc0, lc1 = LC_X0 - x0 - 0.04, LC_X1 - x0 + 0.04
fence_side((x0, yb), (x1, yb), [(0, lc0, "panel"), (lc0, lc1, "open"), (lc1, Lx, "panel")])
fence_side((x1, yb), (x1, yt), [(0, Ly, "panel")])
fence_side((x1, yt), (x0, yt), [(0, x1 - LC_X1 - 0.04, "panel"), (x1 - LC_X1 - 0.04, x1 - LC_X0 + 0.04, "open"),
                                (x1 - LC_X0 + 0.04, Lx, "panel")])
d0, d1 = yt - 1.72, yt - 0.58
t0, t1 = yt - CONV_Y - 0.45, yt - CONV_Y + 0.45
fence_side((x0, yt), (x0, yb), [(0, d0, "panel"), (d0, d1, "door"), (d1, t0, "panel"), (t0, t1, "tunnel"),
                                (t1, Ly, "panel")])

# tunnel polycarbonate sur l'entrée convoyeur (ISO 13857)
tn = []
for sy in (-0.42, 0.42):
    tn.append(box((1.2, 0.006, 0.75), PC, (0, sy, CONV_Z + 0.36), cs=False))
    tn.append(profile((1.2, 0.03, 0.03), (0, sy, CONV_Z + 0.73)))
    tn.append(profile((0.03, 0.03, CONV_Z + 0.74), (-0.6, sy, (CONV_Z + 0.74) / 2)))
tn.append(box((1.2, 0.84, 0.006), PC, (0, 0, CONV_Z + 0.74), cs=False))
w(solid("tunnel protection convoyeur", (x0 - 0.6, CONV_Y, 0), tn))

# porte d'accès (pivote vers l'extérieur) + interrupteur de sécurité à interverrouillage
door = [profile((0.045, 0.045, 2.0), (0, 0.04, 1.12)), profile((0.045, 0.045, 2.0), (0, 1.08, 1.12)),
        profile((0.045, 1.0, 0.045), (0, 0.56, 0.14)), profile((0.045, 1.0, 0.045), (0, 0.56, 2.1)),
        profile((0.045, 1.0, 0.045), (0, 0.56, 1.0)),
        box((0.006, 0.99, 0.82), PC, (0, 0.56, 0.58), cs=False), box((0.006, 0.99, 1.04), PC, (0, 0.56, 1.56), cs=False),
        cyl(0.012, 0.25, BLK, (-0.06, 0.95, 1.05)), box((0.05, 0.02, 0.02), BLK, (-0.035, 0.95, 1.17)),
        box((0.05, 0.02, 0.02), BLK, (-0.035, 0.95, 0.93))]
w(solid("porte acces maintenance", (x0, yt - d1, 0), ['DEF PORTE Pose { children [ %s ] }' % render(door)]))
w(solid("poteaux porte", (x0, 0, 0), [profile((0.045, 0.045, 2.2), (0, yt - d1, 1.1)),
                                      profile((0.045, 0.045, 2.2), (0, yt - d0, 1.1)),
                                      profile((0.045, yt - d0 - (yt - d1), 0.045), (0, (2 * yt - d0 - d1) / 2, 2.18))]))
w(solid("interrupteur securite porte (ISO 14119)", (x0 - 0.05, yt - d0 - 0.06, 1.1), [
    box((0.05, 0.05, 0.16), YEL), box((0.025, 0.025, 0.025), app("0.1 0.8 0.1", emissive="0 0.6 0", def_="LED_PORTE"),
                                      (-0.03, 0, 0.05))]))
# panneaux de signalisation (danger robot) sur le carter
for (px, py, yawp) in ((x0 - 0.01, -0.8, math.pi / 2), (0.0, yb - 0.01, 0), (x1 + 0.01, 0.0, math.pi / 2)):
    w(solid("panneau danger %g %g" % (px, py), (px, py, 1.6), [
        box((0.004, 0.3, 0.3), YEL, (0, 0, 0), (0, 0, 1, yawp + math.pi / 2)),
        box((0.006, 0.2, 0.05), BLK, (0, 0, -0.08), (0, 0, 1, yawp + math.pi / 2))]))

# =============================================================== barrières immatérielles (type SICK deTec) + muting
def light_curtain(name, y, def_prefix):
    parts = []
    for xx in (LC_X0, LC_X1):
        parts.append(profile((0.06, 0.06, 2.15), (xx, y, 1.075)))
        parts.append(box((0.04, 0.035, 1.75), YEL, (xx + (0.045 if xx == LC_X0 else -0.045), y, 1.1)))
        parts.append(box((0.006, 0.03, 1.7), BLK, (xx + (0.066 if xx == LC_X0 else -0.066), y, 1.1)))
    parts.append(box((0.05, 0.05, 0.06), app("1 1 1", emissive="0 0 0", def_=def_prefix + "_LAMPE_MUTING"),
                     (LC_X0, y + 0.06, 2.2)))
    beams = render(box((LC_X1 - LC_X0 - 0.14, 0.004, 0.004),
                         app("1 0 0", emissive="1 0 0", transp=0.6), ((LC_X0 + LC_X1) / 2, y, z), cs=False) for z in LC_Z)
    parts.append('DEF %s_FAISCEAUX Pose { children [ %s ] }' % (def_prefix, beams))
    w(solid(name, (0, 0, 0), parts))


light_curtain("barriere immaterielle sortie palettes (LC1)", yb, "LC1")
light_curtain("barriere immaterielle appro palettes (LC2)", yt, "LC2")
mut = []
for yy in (-3.9, -4.5):
    for xx in (LC_X0 - 0.06, LC_X1 + 0.06):
        mut.append(profile((0.045, 0.045, 0.45), (xx, yy, 0.225)))
    mut.append(box((0.03, 0.05, 0.06), app("0.95 0.6 0.1"), (LC_X0 - 0.03, yy, 0.35)))
    mut.append(box((0.005, 0.05, 0.05), app("0.9 0.1 0.1", 0.2), (LC_X1 + 0.035, yy, 0.35)))
w(solid("capteurs muting LC1", (0, 0, 0), mut))

# =============================================================== colonnes lumineuses, AU, pupitre
def beacon(name, pos, pre):
    parts = [cyl(0.02, 0.4, DARK, (0, 0, 0.2))]
    for i, (c, e) in enumerate((("0.1 0.5 0.1", "G"), ("0.6 0.4 0.05", "O"), ("0.5 0.05 0.05", "R"))):
        parts.append('Pose { translation 0 0 %g children [ Shape { appearance %s geometry Cylinder { radius 0.045 height 0.09 } } ] }'
                     % (0.45 + 0.1 * i, app(c, 0.3, 0, emissive="0 0 0", transp=0.1, def_=pre + "_" + e)))
    w(solid(name, pos, parts))


beacon("colonne lumineuse porte", (x0, yt - d0 + 0.2, 2.2), "FEU1")
beacon("colonne lumineuse sortie", (LC_X1, yb, 2.15), "FEU2")


def estop(name, pos, rot=0):
    w(solid(name, pos, [box((0.1, 0.1, 0.1), YEL), cyl(0.035, 0.04, RED, (0.06, 0, 0), (0, 1, 0, math.pi / 2)),
                        profile((0.045, 0.045, 1.2), (0.0, 0, -0.62))], rot=(0, 0, 1, rot)))


estop("arret urgence sortie", (LC_X1 + 0.35, yb - 0.1, 1.25), -math.pi / 2)
estop("arret urgence convoyeur", (x0 - 0.15, CONV_Y - 0.75, 1.25), math.pi)
estop("arret urgence banderoleuse", (x1 + 0.1, ST["P2"], 1.25), 0)
w(solid("pupitre operateur", (x0 - 0.7, 2.4, 0), [
    box((0.5, 0.4, 1.0), RAL7035, (0, 0, 0.5)),
    box((0.55, 0.45, 0.35), RAL7035, (0, 0, 1.15), (0, 1, 0, -0.35)),
    cyl(0.04, 0.05, RED, (0.14, -0.12, 1.33), (0, 1, 0, -0.35)),
    cyl(0.05, 0.02, YEL, (0.13, -0.12, 1.31), (0, 1, 0, -0.35)),
    cyl(0.025, 0.04, BLUE, (0.14, 0.12, 1.33), (0, 1, 0, -0.35))]))
w('FireExtinguisher { translation %s name "extincteur cellule" }' % f((x0 - 0.5, -3.0, 0)))
w('FireExtinguisher { translation %s name "extincteur stockage" }' % f((6.4, -10.9, 0)))

# =============================================================== zone de stockage du hangar
st = []
for sx in SPOTS:
    for dx in (-0.36, 0.36):
        st.append(box((0.1, 1.25, PAL_Z), app("0.2 0.35 0.7", 0.5, 0.5), (sx + dx, SPOT_Y, PAL_Z / 2)))
    st.append(box((0.9, 0.05, 0.004), YEL, (sx, SPOT_Y + 0.67, 0.002), cs=False))
    st.append(box((0.9, 0.05, 0.004), YEL, (sx, SPOT_Y - 0.67, 0.002), cs=False))
w(solid("emplacements stockage au sol (longrines)", (0, 0, 0), st))


WOOD = app("0.72 0.56 0.36", 0.8, 0)
CARTON = app("0.66 0.49 0.31", 0.85, 0)
FILM = app("0.85 0.9 0.95", 0.1, 0, transp=0.6)


def pallet_prims(x, y, z):
    """Palette Euro 1200x800 (grand côté selon y) en primitives fusionnables."""
    p = []
    for bx in (-0.36, 0.0, 0.36):
        p.append(box((0.08, 1.2, 0.0144), WOOD, (x + bx, y, z + 0.0072)))
        for by in (-0.54, 0.0, 0.54):
            p.append(box((0.1, 0.12, 0.1), WOOD, (x + bx, y + by, z + 0.0644)))
    for by in (-0.54, 0.0, 0.54):
        p.append(box((0.8, 0.12, 0.0144), WOOD, (x, y + by, z + 0.1224)))
    for i in range(6):
        p.append(box((0.08, 1.2, 0.0144), WOOD, (x + 0.72 * (i / 5 - 0.5), y, z + 0.137)))
    return p


def deco_load(x, y, z, h=1.2):
    """Palette chargée de décor (colis filmés) - fusionnée avec tout le décor."""
    p = pallet_prims(x, y, z)
    for k in range(int(round(h / 0.3))):
        for hx in (-0.2, 0.2):
            for hy in (-0.3, 0.3):
                p.append(box((0.395, 0.595, 0.295), CARTON, (x + hx, y + hy, z + 0.144 + 0.15 + 0.3 * k)))
    p.append(box((0.82, 1.22, h + 0.01), FILM, (x, y, z + 0.144 + h / 2), cs=False))
    return p


DECO = []
# racks à palettes le long du mur sud (stockage en hauteur, décor)
RY0, RY1 = BY0 + 0.4, BY0 + 1.55
rk = []
bays = [6.4 + 2.8 * i for i in range(5)]
for i in range(len(bays) + 1):
    xx = bays[0] - 1.4 + 2.8 * i
    for yy in (RY0, RY1):
        rk.append(box((0.09, 0.07, 6.3), app("0.08 0.25 0.55", 0.5, 0.3), (xx, yy, 3.15)))
    for zz in [0.4 + 0.9 * k for k in range(7)]:
        rk.append(box((0.03, RY1 - RY0, 0.03), app("0.08 0.25 0.55", 0.5, 0.3), (xx, (RY0 + RY1) / 2, zz)))
    rk.append(box((0.25, 0.2, 0.4), YEL, (xx, RY1 + 0.12, 0.2)))                       # protection d'échelle
LEV = [0.0, 1.95, 3.9, 5.85]
for zl in LEV[1:]:
    for yy in (RY0, RY1):
        rk.append(box((2.8 * len(bays), 0.05, 0.13), RAL2004, (bays[0] - 1.4 + 1.4 * len(bays), yy, zl - 0.065)))
w(solid("racks a palettes", (0, 0, 0), rk))
import random
random.seed(4)
for bx in bays:
    for zl in LEV:
        for j in (-0.9, 0.0, 0.9):
            if random.random() < 0.72:
                DECO.append(deco_load(bx + j, (RY0 + RY1) / 2, zl, random.choice((0.9, 1.2, 1.2, 1.5))))
for sx in SPOTS[-2:]:
    DECO.append(deco_load(sx, SPOT_Y, PAL_Z))
for i, (xx, yy) in enumerate(((12.0, 4.5), (13.2, 4.5), (14.4, 4.5), (9.0, 4.5))):
    DECO.append(deco_load(xx, yy, 0.0, 1.2 if i % 2 else 0.9))
for k in range(10):
    DECO.append(pallet_prims(16.5, 4.5, 0.144 * k))
w(solid("stock palettes (decor)", (0, 0, 0), DECO))

# =============================================================== chariot élévateur
w('DEF FENWICK Forklift { translation 1.8 -9.5 0.81 rotation 0 0 1 1.5708 name "chariot elevateur" enablePhysics FALSE }')

# =============================================================== automate de cellule (capteurs = devices)
def dsensor(name, pos, yaw, rng, typ="generic"):
    return ('DistanceSensor { translation %s rotation 0 0 1 %g name "%s" lookupTable [ 0 0 0, %g %g 0 ] type "%s" }'
            % (f(pos), yaw, name, rng, rng, typ))


dev = []
cy_em = CONV_Y + 0.40
for name, x in (("B1_annonce_colis", -7.5), ("B5_accumulation", -3.0), ("B2_comptage_sas", -0.58),
                ("B4_lot_colis2", -0.25), ("B3_lot_colis1", 0.25)):
    dev.append(dsensor(name, (x, cy_em, 0.9), -math.pi / 2, 1.0))
for name, y in (("S_P0_magasin", ST["P0"]), ("S_P1_palettisation", ST["P1"]), ("S_P2_banderoleuse", ST["P2"]),
                ("S_P3_evacuation", ST["P3"])):
    dev.append(dsensor(name, (PAL_X - 0.55, y, 0.17), 0, 1.0))
dev.append(dsensor("M1_muting", (LC_X0, -3.9, 0.35), 0, 1.3))
dev.append(dsensor("M2_muting", (LC_X0, -4.5, 0.35), 0, 1.3))
for pre, y in (("LC1", yb), ("LC2", yt)):
    for i, z in enumerate(LC_Z):
        dev.append(dsensor("%s_faisceau%d" % (pre, i + 1), (LC_X0 + 0.07, y, z), 0, 1.3, "laser"))
for lname, lpos, lyaw in (("scrutateur_laser_est", (3.75, -4.40, 0.25), -2.3562),
                          ("scrutateur_laser_ouest", (-0.15, -4.40, 0.25), -0.7854)):
    NR, FOV = 31, 1.65
    for kk in range(NR):
        a = lyaw + FOV / 2 - kk * FOV / (NR - 1)
        dev.append(dsensor("%s_%02d" % (lname, kk), lpos, a, 8.0))
    dev.append(solid("tete " + lname, lpos, [cyl(0.07, 0.16, YEL), cyl(0.06, 0.05, BLK, (0, 0, 0.1))]))
    dev.append(solid("support " + lname, (lpos[0], lpos[1], 0.08), [box((0.1, 0.1, 0.16), DARK)]))
dev.append("""Display {
      translation %s rotation 0 1 0 -0.35
      name "IHM"
      width 320 height 200
      children [ Shape { appearance PBRAppearance { baseColor 1 1 1 roughness 1 metalness 0
                   baseColorMap ImageTexture { url [ ] } emissiveColor 0.6 0.6 0.6 }
                 geometry Box { size 0.005 0.36 0.225 } } ]
    }""" % f((x0 - 0.7 + 0.17, 2.4 + 0.02, 1.2)))
dev.append('Emitter { name "plc_tx" channel 1 }')
dev.append('Receiver { name "robot_rx" channel 2 }')
w("""DEF API Robot {
  name "automate cellule"
  controller "cell_plc"
  supervisor TRUE
  children [
    %s
  ]
}""" % "\n    ".join(dev))


with open(OUT, "w") as fh:
    fh.write("\n".join(out) + "\n")
print("écrit", os.path.abspath(OUT), len(out), "noeuds")
