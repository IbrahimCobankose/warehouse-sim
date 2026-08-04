#!/usr/bin/env bash
# Projeyi PX4 kurulumuna bağlar. Tekrar tekrar çalıştırılabilir (idempotent).
#
# NEDEN SEMBOLİK BAĞ: PX4'ün px4-rc.gzsim betiği world'ü
# "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf" olarak arıyor ve bu değişkenleri
# gz_env.sh koşulsuz olarak PX4'ün kendi dizinlerine ayarlıyor -- yani dışarıdan
# env ile göstermek işe yaramıyor. Dosyaları PX4'e kopyalamak yerine bağlıyoruz
# ki proje tek kaynak olarak kalsın; yeniden üretince PX4 tarafı da güncellensin.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
AIRFRAMES="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME_ID="4022_gz_warehouse_scout"
VEHICLE_MODEL="warehouse_scout"

log() { printf '  %s\n' "$*"; }

[ -d "$PX4_DIR" ] || { echo "HATA: PX4 bulunamadı: $PX4_DIR" >&2; exit 1; }
[ -d "$GZ_MODELS" ] || { echo "HATA: PX4 gz model dizini yok: $GZ_MODELS" >&2; exit 1; }

echo "PX4 entegrasyonu: $PX4_DIR"

# ---------------------------------------------------------------- 1. gz shim
# ROS Jazzy'nin gz_tools_vendor'ı PATH'te sistem gz'sini gölgeliyor ve kendi
# GZ_CONFIG_PATH'i olmadığı için "gz sim" çalışmıyor. PX4'ün başlatma betiği
# doğrudan `gz sim --versions` çağırdığı için bu PX4'ü de düşürüyor.
# Çözüm: sadece gz'yi hedefleyen bir shim dizini, PATH'in başına eklenir.
SHIM_DIR="$PROJECT_DIR/scripts/bin"
mkdir -p "$SHIM_DIR"
if [ -x /usr/bin/gz ]; then
    ln -sfn /usr/bin/gz "$SHIM_DIR/gz"
    log "gz shim      -> $SHIM_DIR/gz -> /usr/bin/gz"
else
    echo "HATA: /usr/bin/gz yok. gz-harmonic kurulu mu?" >&2; exit 1
fi

# ------------------------------------------------------- 2. model + world bağı
link() {
    local src="$1" dst="$2"
    if [ -e "$dst" ] && [ ! -L "$dst" ]; then
        echo "HATA: $dst gerçek bir dosya/dizin, üzerine yazmıyorum." >&2
        exit 1
    fi
    ln -sfn "$src" "$dst"
    log "$(basename "$dst")"
}

# Eski model adından kalan bağlar: bırakılırsa PX4'ün gz hedef üreticisi
# "gz_x500_warehouse" adını iki kez oluşturmaya çalışıp CMake'i düşürür.
for stale in x500_warehouse; do
    [ -L "$GZ_MODELS/$stale" ] && { rm -f "$GZ_MODELS/$stale"; log "eski bağ silindi: $stale"; }
done
rm -f "$AIRFRAMES/4022_gz_x500_warehouse"
sed -i '/4022_gz_x500_warehouse/d' "$AIRFRAMES/CMakeLists.txt"

log "modeller:"
link "$PROJECT_DIR/gz/models/$VEHICLE_MODEL"   "$GZ_MODELS/$VEHICLE_MODEL"
link "$PROJECT_DIR/gz/models/warehouse_assets" "$GZ_MODELS/warehouse_assets"
log "world:"
link "$PROJECT_DIR/gz/worlds/warehouse.sdf"   "$GZ_WORLDS/warehouse.sdf"

# ------------------------------------------------------------- 3. airframe
# Airframe ROMFS'e gömüldüğü için bağ değil kopya gerekiyor; build sırasında
# içeriği okunuyor.
cp "$PROJECT_DIR/px4/airframes/$AIRFRAME_ID" "$AIRFRAMES/$AIRFRAME_ID"
chmod +x "$AIRFRAMES/$AIRFRAME_ID"
log "airframe     -> $AIRFRAMES/$AIRFRAME_ID"

CMAKE_LIST="$AIRFRAMES/CMakeLists.txt"
if grep -q "$AIRFRAME_ID" "$CMAKE_LIST"; then
    log "CMakeLists   -> zaten kayıtlı"
else
    # 4021_gz_x500_flow satırının hemen ardına ekle
    sed -i "s/^\(\s*\)4021_gz_x500_flow\s*$/&\n\1$AIRFRAME_ID/" "$CMAKE_LIST"
    grep -q "$AIRFRAME_ID" "$CMAKE_LIST" \
        || { echo "HATA: CMakeLists'e airframe eklenemedi." >&2; exit 1; }
    log "CMakeLists   -> $AIRFRAME_ID eklendi"
fi

echo
echo "Tamam. Airframe ROMFS'e girmesi için PX4'ü yeniden derleyin:"
echo "    cd $PX4_DIR && make px4_sitl_default"
