#!/usr/bin/env bash
# Tüm simülasyon hattını TEK KOMUTLA açar (gnome-terminal sekmeleri).
#
#   ./scripts/start.sh                    # sim + lokalizasyon + tarama, sonra uçuş
#   ./scripts/start.sh --view             # canlı ön kamera penceresini de aç
#   ./scripts/start.sh --run run2         # bu koşuyu out/scans_run2'ye yaz (KPI tekrarı)
#   ./scripts/start.sh --no-fly           # sadece hattı kur, uçuşu ben başlatayım
#   ./scripts/start.sh --route koridor1_A_seviye2
#   ./scripts/start.sh --no-barcode       # barkod hattını kapat
#
# Sekmeler kendi ön koşullarını BEKLER: lokalizasyon sim ayağa kalkmadan,
# tarama da EV beslemesi akmadan başlamaz -- elle sıralama derdi yok.
# Uçuş bu terminalde koşar ki çıktısı gözünün önünde olsun.

set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
PY="$PROJECT_DIR/.venv/bin/python"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
EV_CSV="$PROJECT_DIR/out/logs/ev_feed.csv"
# ROS'un gz_tools_vendor'ı sistem gz'sini gölgeliyor; shim PATH'in başına.
export PATH="$PROJECT_DIR/scripts/bin:$PATH"

# ------------------------------------------------------------- bekleme yardımcıları
wait_for_sim() {                      # gz world'ü konuşmaya başlayana kadar
    echo "[bekle] Gazebo/PX4 ayağa kalkıyor..."
    for _ in $(seq 1 180); do
        if gz topic -l 2>/dev/null | grep -q '^/world/warehouse'; then
            echo "[tamam] sim ayakta."; return 0
        fi
        sleep 1
    done
    echo "[HATA] sim 180 s'de açılmadı -- 1. sekmedeki çıktıya bak." >&2
    return 1
}

wait_for_ev() {                       # ev_feed.csv AKMAYA başlayana kadar
    echo "[bekle] EV beslemesi (apriltag_localize)..."
    for _ in $(seq 1 180); do
        # Sadece dosyanın varlığı yetmez: önceki koşudan kalmış olabilir.
        if [ -s "$EV_CSV" ] && [ "$(find "$EV_CSV" -newermt '-5 seconds' 2>/dev/null)" ]; then
            echo "[tamam] EV besleme akıyor."; return 0
        fi
        sleep 1
    done
    echo "[HATA] EV beslemesi gelmedi -- 2. sekmeye bak. Bu olmadan" >&2
    echo "       build_inventory her okumayı 'poz yok' diye düşürür." >&2
    return 1
}

# ----------------------------------------------------- sekme içi aşamalar (dahilî)
# gnome-terminal sekmeleri betiği bu modlarla yeniden çağırır.
case "${_STAGE:-}" in
  sim)  exec ./scripts/run_sim.sh ;;
  loc)  wait_for_sim || { echo "Enter ile kapat"; read -r; exit 1; }
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
        exec "$PY" scripts/apriltag_localize.py ;;
  scan) wait_for_ev || { echo "Enter ile kapat"; read -r; exit 1; }
        source "$ROS_SETUP"
        exec "$PY" scripts/scan_boxes.py --no-bridge ${SCAN_ARGS:-} ;;
  view) wait_for_ev || { echo "Enter ile kapat"; read -r; exit 1; }
        source "$ROS_SETUP"
        exec "$PY" scripts/view_front.py --no-bridge ;;
esac

# --------------------------------------------------------------------- argümanlar
ROUTE="tam_tur_geo"
RUN=""
WANT_VIEW=0
WANT_FLY=1
BARCODE="--with-barcode"
while [ $# -gt 0 ]; do
    case "$1" in
        --view)       WANT_VIEW=1 ;;
        --no-fly)     WANT_FLY=0 ;;
        --no-barcode) BARCODE="" ;;
        --run)        RUN="$2"; shift ;;
        --route)      ROUTE="$2"; shift ;;
        -h|--help)    sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "Bilinmeyen argüman: $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done

SCAN_OUT="out/scans${RUN:+_$RUN}"
export SCAN_ARGS="$BARCODE --out $SCAN_OUT"

# ------------------------------------------------------------------- ön kontroller
fail() { echo "[HATA] $*" >&2; exit 1; }

[ -x "$PY" ]        || fail ".venv yok/bozuk: $PY"
[ -f "$ROS_SETUP" ] || fail "ROS Jazzy bulunamadı: $ROS_SETUP"
command -v gnome-terminal >/dev/null || fail "gnome-terminal yok."
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] \
    || fail "PX4 derlenmemiş: cd $PX4_DIR && make px4_sitl_default"

# Klasör TAŞINDIYSA/YENİDEN ADLANDIRILDIYSA PX4'teki sembolik bağlar kırılır.
# Bunu burada yakalamak, dünyanın sessizce yanlış yüklenmesinden iyidir.
WORLD_LINK="$PX4_DIR/Tools/simulation/gz/worlds/warehouse.sdf"
if [ ! -e "$WORLD_LINK" ]; then
    fail "PX4'teki world bağı kırık ($WORLD_LINK).
       Proje klasörü taşındıysa şunu koş: ./scripts/setup_px4_integration.sh"
fi

# BAYAT gz sim SUNUCUSU: PX4 kapansa da ayakta kalıp sonraki PX4'ü bozuyor.
if pgrep -f "gz[ ]sim|bin/px4" >/dev/null 2>&1; then
    echo "Çalışan sim süreçleri bulundu:"
    pgrep -af "gz[ ]sim|bin/px4" | sed 's/^/  /'
    read -r -p "Kapatılsın mı? [E/h] " a
    case "${a:-e}" in
        [hHnN]*) echo "Bırakıldı -- bayat sunucu bağlanma riski sende." ;;
        *) pkill -9 -f "gz[ ]sim" 2>/dev/null
           pkill -9 -f "bin/px4" 2>/dev/null
           sleep 2; echo "Kapatıldı." ;;
    esac
fi

mkdir -p out/logs
rm -f "$EV_CSV"          # bayat EV dosyası "akıyor" sanılmasın

# ------------------------------------------------------------------ sekmeleri aç
echo "== sekmeler açılıyor: sim | lokalizasyon | tarama${WANT_VIEW:+ | kamera} =="
echo "   tarama çıktısı: $SCAN_OUT"

ARGS=( --window --title=sim -- bash -c "cd '$PROJECT_DIR' && _STAGE=sim ./scripts/start.sh; echo; echo '[sim bitti]'; exec bash" )
ARGS+=( --tab --title=lokalizasyon -- bash -c "cd '$PROJECT_DIR' && _STAGE=loc ./scripts/start.sh; echo; echo '[lokalizasyon bitti]'; exec bash" )
ARGS+=( --tab --title=tarama -- bash -c "cd '$PROJECT_DIR' && _STAGE=scan SCAN_ARGS='$SCAN_ARGS' ./scripts/start.sh; echo; echo '[tarama bitti]'; exec bash" )
[ "$WANT_VIEW" = 1 ] && \
ARGS+=( --tab --title=kamera -- bash -c "cd '$PROJECT_DIR' && _STAGE=view ./scripts/start.sh; echo; exec bash" )

gnome-terminal "${ARGS[@]}" || fail "gnome-terminal sekmeleri açılamadı."

# ------------------------------------------------------------------------- uçuş
wait_for_sim || exit 1
cat <<'NOTE'

  >> 1. SEKMEYİ KONTROL ET: konsolda "Number of good matches" ~15-20 olmalı.
     2 civarındaysa zemin dokusu yüklenmemiştir -- UÇMA, sim'i yeniden kur.
NOTE
wait_for_ev || exit 1

if [ "$WANT_FLY" = 0 ]; then
    echo
    echo "Hat hazır. Uçuş için:"
    echo "  bash scripts/flight_logged.sh $ROUTE --routes config/route_gen.yaml"
    exit 0
fi

echo
read -r -p "Uçuşu başlat ($ROUTE, ~13 dk)? [E/h] " a
case "${a:-e}" in [hHnN]*) echo "Atlandı."; exit 0 ;; esac

bash scripts/flight_logged.sh "$ROUTE" --routes config/route_gen.yaml
rc=$?

echo
echo "Uçuş bitti. Şimdi TARAMA sekmesinde Ctrl-C (özet ve son yazımlar orada basılır),"
echo "sonra çözümleme:"
echo "  ./scripts/analyze.sh${RUN:+ --run $RUN}"
exit $rc
