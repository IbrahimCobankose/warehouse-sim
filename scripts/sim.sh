#!/usr/bin/env bash
# 1/3 -- ORTAMI AÇAR: sim + lokalizasyon + tarama (isteğe bağlı canlı kamera).
# Uçurmaz, çözümlemez. Uçuş: scripts/fly.sh   Çözümleme: scripts/analyze.sh
#
#   ./scripts/sim.sh                 # üç sekme
#   ./scripts/sim.sh --view          # canlı ön kamera sekmesini de aç
#   ./scripts/sim.sh --run run4      # tarama çıktısı out/scans_run4 (KPI tekrarı)
#   ./scripts/sim.sh --no-barcode    # barkod hattını kapat
#   ./scripts/sim.sh --attach        # SİM ZATEN AÇIK: ona bağlan, yeniden başlatma
#
# Sekmeler kendi ön koşullarını BEKLER: lokalizasyon sim ayağa kalkmadan,
# tarama da kamera köprüsü açılmadan başlamaz. Sekmeler açık kalır; hata olursa
# ilgili sekmede durur, kaybolmaz.

set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
PY="$PROJECT_DIR/.venv/bin/python"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
# ROS'un gz_tools_vendor'ı sistem gz'sini gölgeliyor; shim PATH'in başına.
export PATH="$PROJECT_DIR/scripts/bin:$PATH"

# shellcheck source=scripts/lib_wait.sh
source "$PROJECT_DIR/scripts/lib_wait.sh"

# ----------------------------------------------------- sekme içi aşamalar (dahilî)
case "${_STAGE:-}" in
  sim)  exec ./scripts/run_sim.sh ;;
  loc)  wait_for_sim || { echo "Enter ile kapat"; read -r; exit 1; }
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
        exec "$PY" scripts/apriltag_localize.py ;;
  scan) wait_for_localize && wait_for_camera_topic || { echo "Enter ile kapat"; read -r; exit 1; }
        source "$ROS_SETUP"
        exec "$PY" scripts/scan_boxes.py --no-bridge ${SCAN_ARGS:-} ;;
  view) wait_for_localize && wait_for_camera_topic || { echo "Enter ile kapat"; read -r; exit 1; }
        source "$ROS_SETUP"
        exec "$PY" scripts/view_front.py --no-bridge ;;
esac

# --------------------------------------------------------------------- argümanlar
RUN=""; WANT_VIEW=0; BARCODE="--with-barcode"; ATTACH=0
while [ $# -gt 0 ]; do
    case "$1" in
        --view)       WANT_VIEW=1 ;;
        --no-barcode) BARCODE="" ;;
        --attach)     ATTACH=1 ;;
        --run)        RUN="$2"; shift ;;
        -h|--help)    sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "Bilinmeyen argüman: $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done
export SCAN_ARGS="$BARCODE --out out/scans${RUN:+_$RUN}"

# ------------------------------------------------------------------- ön kontroller
fail() { echo "[HATA] $*" >&2; exit 1; }
[ -x "$PY" ]        || fail ".venv yok/bozuk: $PY"
[ -f "$ROS_SETUP" ] || fail "ROS Jazzy bulunamadı: $ROS_SETUP"
command -v gnome-terminal >/dev/null || fail "gnome-terminal yok."
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] \
    || fail "PX4 derlenmemiş: cd $PX4_DIR && make px4_sitl_default"
# Klasör taşındıysa PX4'teki sembolik bağlar kırılır; dünya sessizce yanlış
# yüklenmesin diye burada yakalanıyor.
[ -e "$PX4_DIR/Tools/simulation/gz/worlds/warehouse.sdf" ] \
    || fail "PX4'teki world bağı kırık. Proje klasörü taşındıysa:
       ./scripts/setup_px4_integration.sh"

# BAYAT gz sim SUNUCUSU: PX4 kapansa da ayakta kalıp sonraki PX4'ü bozuyor.
# --attach ile bilerek çalışan sim'e bağlanılıyor, o zaman sorma.
if [ "$ATTACH" = 0 ] && pgrep -f "gz[ ]sim|bin/px4" >/dev/null 2>&1; then
    echo "Çalışan sim süreçleri bulundu:"
    pgrep -af "gz[ ]sim|bin/px4" | sed 's/^/  /'
    read -r -p "Kapatılsın mı? [E/h] " a
    case "${a:-e}" in
        [hHnN]*) echo "Bırakıldı -- bayat sunucuya bağlanma riski sende." ;;
        *) pkill -9 -f "gz[ ]sim" 2>/dev/null; pkill -9 -f "bin/px4" 2>/dev/null
           sleep 2; echo "Kapatıldı." ;;
    esac
fi
mkdir -p out/logs

# ------------------------------------------------------------------ sekmeleri aç
stage_tab() {   # stage_tab <başlık> <_STAGE>
    printf '%s\0' --tab --title="$1" -- bash -c \
        "cd '$PROJECT_DIR' && _STAGE=$2 SCAN_ARGS='$SCAN_ARGS' ./scripts/sim.sh; \
         echo; echo '[$1 bitti -- sekme açık kaldı]'; exec bash"
}

if [ "$ATTACH" = 1 ]; then          # sim zaten açık -> yalnız lokalizasyon sekmesiyle başla
    ARGS=( --window --title=lokalizasyon -- bash -c \
           "cd '$PROJECT_DIR' && _STAGE=loc ./scripts/sim.sh; echo; echo '[lokalizasyon bitti]'; exec bash" )
else
    ARGS=( --window --title=sim -- bash -c \
           "cd '$PROJECT_DIR' && _STAGE=sim ./scripts/sim.sh; echo; echo '[sim bitti]'; exec bash" )
fi
[ "$ATTACH" = 0 ] && \
    while IFS= read -r -d '' a; do ARGS+=( "$a" ); done < <(stage_tab lokalizasyon loc)
while IFS= read -r -d '' a; do ARGS+=( "$a" ); done < <(stage_tab tarama scan)
if [ "$WANT_VIEW" = 1 ]; then
    while IFS= read -r -d '' a; do ARGS+=( "$a" ); done < <(stage_tab kamera view)
fi

if [ "$ATTACH" = 1 ]; then
    echo "== çalışan sim'e bağlanılıyor -- sekmeler: lokalizasyon | tarama${WANT_VIEW:+ | kamera} =="
else
    echo "== sekmeler: sim | lokalizasyon | tarama${WANT_VIEW:+ | kamera} =="
fi
echo "   tarama çıktısı: out/scans${RUN:+_$RUN}"
gnome-terminal "${ARGS[@]}" || fail "gnome-terminal sekmeleri açılamadı."

cat <<NOTE

  >> 1. SEKME: "Number of good matches" ~15-20 olmalı. 2 civarındaysa zemin
     dokusu yüklenmemiştir -- UÇMA.
  >> 2. SEKME: "EV besleme kaydı" satırını görünce lokalizasyon ayakta.
     Yerde "EV BOŞLUK ... tag YOK" uyarıları NORMAL: araç kalkmadan tag
     görünmüyor, pozlar kalkıştan sonra akmaya başlar.

Hazır olunca uçuş (AYRI terminal):
  ./scripts/fly.sh${RUN:+ --run $RUN}
NOTE
