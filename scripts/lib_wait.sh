#!/usr/bin/env bash
# Ortak bekleme yardımcıları. sim.sh ve fly.sh aynı ön koşulları denetlesin
# diye tek yerde: iki betiğin "hazır" tanımı ayrışırsa biri diğerinin
# görmediği bir duruma uçar.
#
# Çağıran betik PATH'e scripts/bin shim'ini eklemiş olmalı (gz için).

: "${EV_CSV:?lib_wait: EV_CSV tanımlı olmalı}"

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

wait_for_ev() {                       # ev_feed.csv AKMAYA başlayana kadar
    local tries="${1:-180}"
    echo "[bekle] EV beslemesi (apriltag_localize)..."
    for _ in $(seq 1 "$tries"); do
        # Varlık yetmez: dosya bir önceki koşudan kalmış olabilir. Tazelik
        # mtime ile denetleniyor -- bu yüzden dosyayı SİLMEYE gerek yok.
        if [ -s "$EV_CSV" ] && [ "$(find "$EV_CSV" -newermt '-5 seconds' 2>/dev/null)" ]; then
            echo "[tamam] EV besleme akıyor."; return 0
        fi
        sleep 1
    done
    echo "[HATA] EV beslemesi gelmedi -- lokalizasyon sekmesine bak. Bu olmadan" >&2
    echo "       build_inventory her okumayı 'poz yok' diye düşürür." >&2
    return 1
}
