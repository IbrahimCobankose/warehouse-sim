#!/usr/bin/env bash
# Ortak hazırlık denetimleri. sim.sh ve fly.sh aynı tanımı kullansın diye tek
# yerde: ikisinin "hazır" tanımı ayrışırsa biri diğerinin görmediği bir duruma
# uçar.
#
# ÖLÇÜLDÜ -- "EV BESLEMESİ AKIYOR MU" DİYE BEKLEMEK YANLIŞ. Araç yerdeyken
# alt kamera zeminin ALTINDA (z ~ -0.09) ve hiçbir tag görmüyor; ön kamera da
# kalkıştan önce raf tag'i görmüyor. Ölçüm (35 s, sim ayakta, araç yerde):
# alt kamera 195 kare / 0 poz, ön kamera 151 kare / 0 poz. Yani ev_feed.csv
# kalkıştan ÖNCE yalnız başlık satırını taşır ve asla tazelenmez -- bunu
# beklemek sonsuza kadar bekler. Doğru koşul "poz akıyor mu" değil,
# "lokalizasyon ayakta ve kareleri alıyor mu".
#
# Çağıran betik PATH'e scripts/bin shim'ini eklemiş olmalı (gz için).

FRONT_TOPIC="${FRONT_TOPIC:-/warehouse_scout/camera_front/image}"

wait_for_sim() {                      # gz world'ü konuşmaya başlayana kadar
    local tries="${1:-180}"
    echo "[bekle] Gazebo/PX4 ayağa kalkıyor..."
    for _ in $(seq 1 "$tries"); do
        if gz topic -l 2>/dev/null | grep -q '^/world/warehouse'; then
            echo "[tamam] sim ayakta."; return 0
        fi
        sleep 1
    done
    echo "[HATA] sim ${tries} s'de açılmadı -- sim sekmesindeki çıktıya bak." >&2
    return 1
}

wait_for_localize() {                 # apriltag_localize süreci ayakta mı
    local tries="${1:-180}"
    echo "[bekle] lokalizasyon (apriltag_localize)..."
    for _ in $(seq 1 "$tries"); do
        if pgrep -f "apriltag_localize\.py" >/dev/null 2>&1; then
            echo "[tamam] lokalizasyon ayakta."
            echo "        (EV pozları kalkıştan SONRA akmaya başlar -- yerde"
            echo "         tag görünmüyor, bu normal.)"
            return 0
        fi
        sleep 1
    done
    echo "[HATA] apriltag_localize çalışmıyor -- lokalizasyon sekmesine bak." >&2
    echo "       Elle:  source /opt/ros/jazzy/setup.bash && \\" >&2
    echo "              .venv/bin/python scripts/apriltag_localize.py" >&2
    return 1
}

wait_for_camera_topic() {             # ön kamera köprüsü açıldı mı (--no-bridge için)
    local tries="${1:-90}"
    echo "[bekle] ön kamera konusu ($FRONT_TOPIC)..."
    for _ in $(seq 1 "$tries"); do
        if ros2 topic list 2>/dev/null | grep -qx "$FRONT_TOPIC"; then
            echo "[tamam] kamera konusu yayında."; return 0
        fi
        sleep 1
    done
    echo "[HATA] $FRONT_TOPIC yayında değil. Köprüyü apriltag_localize açar;" >&2
    echo "       o çalışmıyorsa tarama --no-bridge ile kare alamaz." >&2
    return 1
}
