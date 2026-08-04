#!/usr/bin/env bash
# Depo simülasyonunu başlatır: Gazebo + PX4 SITL + 3 kameralı warehouse_scout.
#
#   ./scripts/run_sim.sh              GUI ile
#   HEADLESS=1 ./scripts/run_sim.sh   GUI'siz (kayıt / CI için)
#
# Önce bir kez ./scripts/setup_px4_integration.sh çalıştırılmış olmalı.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

# ROS Jazzy'nin gz_tools_vendor'ı PATH'te sistem gz'sini gölgeliyor ve
# çalışmıyor; PX4'ün px4-rc.gzsim betiği doğrudan `gz sim --versions`
# çağırdığı için bu olmadan başlatma "Gazebo gz sim not found" ile düşer.
export PATH="$PROJECT_DIR/scripts/bin:$PATH"

if ! gz sim --versions >/dev/null 2>&1; then
    echo "HATA: 'gz sim' çalışmıyor. ./scripts/setup_px4_integration.sh çalıştırıldı mı?" >&2
    exit 1
fi

# Çalışan bir Gazebo varsa kapat.
#
# NEDEN: PX4'ün px4-rc.gzsim betiği önce çalışan bir world arıyor ve bulursa
# ONA BAĞLANIYOR -- kendi world'ünü açmıyor ve GUI'yi de hiç başlatmıyor
# (GUI yalnızca sunucuyu kendisi başlattığı dalda açılıyor). Bir önceki
# koşudan kalan sunucu bu yüzden "çalıştırdım ama hiçbir şey açılmadı"
# şeklinde görünür. Ayrıca yanlış world'e bağlanmak sessiz bir hata olur.
#
# Kendi Gazebo'nuzu ayrıca çalıştırıyorsanız KEEP_EXISTING=1 ile atlayın.
if [ -z "${KEEP_EXISTING:-}" ] && pgrep -f "gz[ ]sim" >/dev/null 2>&1; then
    running_world=$(gz topic -l 2>/dev/null | grep -m1 -oE '^/world/[^/]+' | sed 's|/world/||')
    echo "Çalışan Gazebo bulundu${running_world:+ (world: $running_world)}, kapatılıyor..."
    pkill -f "gz[ ]sim" 2>/dev/null || true
    sleep 2
    pkill -9 -f "gz[ ]sim" 2>/dev/null || true
    sleep 1
fi
pkill -f "${BUILD_DIR}/bin/px4" 2>/dev/null || true

# PX4 çıkarken başlattığı Gazebo süreçleri arkada kalmasın.
cleanup() {
    pkill -f "gz[ ]sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

[ -x "$BUILD_DIR/bin/px4" ] || {
    echo "HATA: PX4 derlenmemiş. cd $PX4_DIR && make px4_sitl_default" >&2; exit 1; }
[ -e "$PX4_DIR/Tools/simulation/gz/worlds/warehouse.sdf" ] || {
    echo "HATA: warehouse world'ü bağlanmamış. ./scripts/setup_px4_integration.sh" >&2; exit 1; }

# Kalkış noktası config'den okunur, tek kaynak orası kalsın.
SPAWN=$("$PROJECT_DIR/.venv/bin/python" - "$PROJECT_DIR/config/warehouse.yaml" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))["spawn"]["pose"]
print(f"{p[0]},{p[1]},{p[2]}")
PY
)

export PX4_SIM_MODEL="gz_warehouse_scout"   # -> airframe 4022_gz_warehouse_scout
export PX4_GZ_WORLD="warehouse"
export PX4_GZ_MODEL_POSE="$SPAWN"
export GZ_IP=127.0.0.1

echo "world  : $PX4_GZ_WORLD"
echo "araç   : warehouse_scout (ön / alt / arka kamera)"
echo "kalkış : $SPAWN"
echo "gz     : $(command -v gz) $(gz sim --versions | head -1)"
[ -n "${HEADLESS:-}" ] && echo "mod    : headless"
echo

echo "Durdurmak için Ctrl-C."
echo

# exec KULLANILMIYOR: exec kabuğu değiştirir ve yukarıdaki EXIT trap'i
# çalışmaz, Gazebo süreçleri arkada kalırdı.
cd "$BUILD_DIR/rootfs"
"$BUILD_DIR/bin/px4" "$BUILD_DIR/etc" -s etc/init.d-posix/rcS -d
