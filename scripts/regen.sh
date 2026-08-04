#!/usr/bin/env bash
# config/warehouse.yaml değiştikten sonra world'ü ve aracı yeniden üretir.
#
# PX4 tarafına ayrıca bir şey yapmak GEREKMEZ: setup_px4_integration.sh
# PX4'ün dizinlerine sembolik bağ kurduğu için üretilen dosyalar oraya
# kendiliğinden yansır. İstisnalar en altta.
#
#   ./scripts/regen.sh              world + araç
#   ./scripts/regen.sh --seed 42    farklı yerleşim (domain randomization)

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$PROJECT_DIR/.venv/bin/python"

SEED_ARG=()
if [ "${1:-}" = "--seed" ]; then
    SEED_ARG=(--seed "$2")
fi

echo "=== world ==="
"$PY" "$PROJECT_DIR/tools/gen_world.py" "${SEED_ARG[@]}"
echo
echo "=== araç ==="
"$PY" "$PROJECT_DIR/tools/gen_vehicle.py"
echo
echo "=== okunabilirlik bütçesi ==="
"$PY" "$PROJECT_DIR/tools/gen_labels.py" --budget | tail -20

cat <<'EOF'

Simülasyon çalışıyorsa yeniden başlatın (dosyalar açılışta okunuyor).

Yeniden üretmenin YETMEDİĞİ tek durum: px4/airframes/ altındaki airframe
dosyasını değiştirdiyseniz. O ROMFS'e gömüldüğü için:
    ./scripts/setup_px4_integration.sh
    cd ~/PX4-Autopilot && make px4_sitl_default
EOF
