#!/usr/bin/env bash
# Construit site/ : animation HTML5 Webots d'un cycle complet (scène 3D interactive, lisible dans tout navigateur).
#   tools/build_site.sh            (DEBUT=0 FIN=218 par défaut, en secondes simulées)
# SANS_ENREGISTREMENT=1 tools/build_site.sh  -> ne refait que la mise en page (description, index, vignette)
# Publication sur GitHub Pages :   git subtree push --prefix site origin gh-pages
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
DEBUT=${DEBUT:-0}
FIN=${FIN:-218}
if [ -z "${SANS_ENREGISTREMENT:-}" ]; then
tmp="$ROOT/worlds/.site_record.wbt"
# pas de 40 ms pour alléger l'animation (25 images/s)
sed 's/basicTimeStep 16/basicTimeStep 40/' "$ROOT/worlds/palettisation.wbt" > "$tmp"
# réservoir de noeuds pré-créés (hors champ) : une animation HTML5 n'exporte que les noeuds présents au départ
python3 - "$tmp" <<'PY'
import sys
with open(sys.argv[1], "a") as fh:
    for i in range(1, 61):
        fh.write('DEF COLIS_%d CardboardBox { translation %g -30 -5 name "colis %d" size 0.4 0.6 0.3 }\n' % (i, i * 0.7, i))
    for i in range(1, 5):
        fh.write('DEF PALETTE_%d WoodenPallet { translation %g -34 -5 rotation 0 0 1 1.5708 name "palette %d" }\n' % (i, i * 1.5, i))
    for i in range(1, 3):
        fh.write('DEF FILM_%d Transform { translation %g -38 -5 scale 1 1 0.05 children [ Shape { appearance '
                 'PBRAppearance { baseColor 0.85 0.9 0.95 transparency 0.55 roughness 0.1 metalness 0 } '
                 'geometry Box { size 0.82 1.22 1 } } ] }\n' % (i, i * 1.5))
PY
cat >> "$tmp" <<WBT
Robot {
  name "animation_recorder"
  controller "animation_recorder"
  controllerArgs [ "../../site/palettisation.html" "$DEBUT" "$FIN" ]
  supervisor TRUE
}
WBT
rm -rf "$ROOT"/site/palettisation.{html,css,json,w3d,jpg}
timeout 1200 "$WEBOTS" --batch --mode=fast --minimize --stdout --stderr "$tmp" > "$ROOT/site/.build.log" 2>&1 || true
rm -f "$tmp"
for f in palettisation.html palettisation.json palettisation.w3d; do
  [ -s "$ROOT/site/$f" ] || { echo "✗ site/$f manquant (voir site/.build.log)"; exit 1; }
done
fi
rm -f "$ROOT/site/.build.log"
# description de la page + page d'accueil (redirection) + vignette
python3 - "$ROOT/site/palettisation.html" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
desc = ("<p><strong>Cellule de palettisation robotisée</strong> (Webots R2025a) : un cycle complet, de l'arrivée des colis "
        "au stockage de la palette dans le hangar.<br><br>"
        "ABB IRB 4600-40 + préhenseur à 12 ventouses, sas de formation de lots (2 colis), palette Euro 4 couches, "
        "banderoleuse à plateau tournant, évacuation par barrière immatérielle avec muting, chariot élévateur.<br>"
        "Clic gauche : orbite · clic droit : déplacement · molette : zoom · barre du bas : lecture / avance dans le temps.<br><br>"
        "Code source : <a href='https://github.com/NicolasBrochen/palletizer-cell-webots'>github.com/NicolasBrochen/palletizer-cell-webots</a></p>")
s = re.sub(r'<div class="description-container">.*?</div>', '<div class="description-container">' + desc + '</div>', s, flags=re.S)
s = s.replace("Web Animation: ", "Animation 3D : ")
open(p, "w", encoding="utf-8").write(s)
PY
cat > "$ROOT/site/index.html" <<'HTML'
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="0; url=palettisation.html">
  <title>Cellule de palettisation (Webots)</title>
</head>
<body>
  <p><a href="palettisation.html">Ouvrir l'animation 3D de la cellule de palettisation</a></p>
</body>
</html>
HTML
[ -s "$ROOT/docs/cellule.jpg" ] && ffmpeg -loglevel error -y -i "$ROOT/docs/cellule.jpg" -vf scale=768:-1 -q:v 4 "$ROOT/site/palettisation.jpg"
touch "$ROOT/site/.nojekyll"
du -sh "$ROOT/site"
