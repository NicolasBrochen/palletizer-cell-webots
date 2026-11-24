#!/usr/bin/env bash
# Réenregistre les médias du README depuis worlds/palettisation.wbt (Webots doit pouvoir afficher : rendu des vidéos) :
#   docs/palettisation.mp4   un cycle complet accéléré x4 (vue d'ensemble)
#   docs/prise_lot.mp4       gros plan : prise d'un lot par le préhenseur à ventouses
#   docs/banderolage.mp4     gros plan : banderoleuse à plateau tournant
#   docs/cellule.jpg, docs/prehenseur.jpg, docs/banderoleuse.jpg   images extraites
#   docs/palettisation.gif   aperçu animé pour le README
#   tools/build_media.sh
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-webots}
tmpd=$(mktemp -d)
world="$ROOT/worlds/.media_record.wbt"
trap 'rm -rf "$tmpd" "$world"' EXIT
run() {  # $1 = plans.json
  # vue nommée pour le recadrage, automate en mode --sans-ihm, écran du pupitre retiré
  sed -e 's/^Viewpoint {/DEF VIEWPOINT Viewpoint {/' -e 's/^  controller "cell_plc"/  controller "cell_plc"\n  controllerArgs [ "--sans-ihm" ]/' "$ROOT/worlds/palettisation.wbt" \
    | python3 -c "import re,sys; s=sys.stdin.read(); print(re.sub(r'Display \{.*?name \"IHM\".*?\n    \}', '', s, flags=re.S), end='')" > "$world"
  cat >> "$world" <<WBT
Robot { name "media_recorder" controller "media_recorder" controllerArgs [ "$1" ] supervisor TRUE }
WBT
  timeout 1800 "$WEBOTS" --batch --mode=fast --stdout --stderr "$world" > "$tmpd/webots.log" 2>&1 || true
}
cat > "$tmpd/a.json" <<JSON
{"width": 1280, "height": 720, "clips": [
  {"file": "$tmpd/cycle.mp4", "start": 0, "end": 216, "accel": 4, "position": [-9.5, 6.8, 6.2], "look_at": [2.5, -4.0, 0.3]}
]}
JSON
cat > "$tmpd/b.json" <<JSON
{"width": 1280, "height": 720, "clips": [
  {"file": "$tmpd/prise.mp4", "start": 36.5, "end": 49.0, "accel": 1, "position": [-2.3, -4.3, 2.3], "look_at": [0.6, -0.9, 1.0]},
  {"file": "$tmpd/bande.mp4", "start": 150.0, "end": 176.0, "accel": 2, "position": [-0.6, -6.6, 2.6], "look_at": [1.9, -2.8, 0.9]}
]}
JSON
run "$tmpd/a.json"
run "$tmpd/b.json"
for f in cycle.mp4 prise.mp4 bande.mp4; do
  [ -s "$tmpd/$f" ] || { tail -30 "$tmpd/webots.log"; echo "✗ $f non enregistré"; exit 1; }
done
D="${OUT:-$ROOT/docs}"
mkdir -p "$D"
# réencodage H.264 compact (les fichiers bruts de Webots font plusieurs dizaines de Mo)
enc() { ffmpeg -loglevel error -y -i "$1" -c:v libx264 -crf 27 -preset slow -pix_fmt yuv420p -movflags +faststart -an "$2"; }
enc "$tmpd/cycle.mp4" "$D/palettisation.mp4"
enc "$tmpd/prise.mp4" "$D/prise_lot.mp4"
enc "$tmpd/bande.mp4" "$D/banderolage.mp4"
ffmpeg -loglevel error -y -ss 12 -i "$D/palettisation.mp4" -frames:v 1 -q:v 3 "$D/cellule.jpg"
ffmpeg -loglevel error -y -ss 6.5 -i "$D/prise_lot.mp4" -frames:v 1 -q:v 3 "$D/prehenseur.jpg"
ffmpeg -loglevel error -y -ss 8 -i "$D/banderolage.mp4" -frames:v 1 -q:v 3 "$D/banderoleuse.jpg"
# aperçu animé : 24 s du cycle accéléré, 10 i/s, 640 px
ffmpeg -loglevel error -y -ss 6 -t 24 -i "$D/palettisation.mp4" -vf "fps=10,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle" "$D/palettisation.gif"
ls -la "$D"
