#!/usr/bin/env bash
# Uçuşu OTOMATİK loglayarak koşar -- artık çıktıyı elle kopyalayıp atmaya gerek yok.
# Asistan bu iki dosyayı DOĞRUDAN okur:
#   out/logs/latest.log   -> fly_route konsolu (waypoint, sapma, irtifa, abort)
#   out/state_log.csv     -> gerçek konum izi (t_s,x,y,z,yaw_deg), arka planda log_state
#
#   bash scripts/flight_logged.sh                 # varsayılan: koridor1_tam_tur
#   bash scripts/flight_logged.sh koridor1_A_seviye2 --abort-offset 1.3
#
# Sim + apriltag_localize (lokalizasyon) AYRI terminallerde çalışıyor olmalı.
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
PY="$PROJECT_DIR/.venv/bin/python"
ROUTE="${1:-koridor1_tam_tur}"
shift || true                       # kalan argümanlar fly_route'a geçer

mkdir -p out/logs
TS="$(date +%Y%m%d_%H%M%S)"
LOG="out/logs/flight_${TS}.log"

# Konum kaydedici arka planda (uçuşa dokunmaz, sadece gz gerçek pozunu okur).
"$PY" scripts/log_state.py --quiet &
LOGGER_PID=$!
# Betik biterse/kesilirse loglayıcıyı da durdur.
trap 'kill "$LOGGER_PID" 2>/dev/null' EXIT INT TERM

echo "== uçuş: $ROUTE   log -> $LOG  + out/state_log.csv =="
# Konsolu hem ekrana hem dosyaya. pipefail sayesinde fly_route'un çıkışı korunur.
"$PY" scripts/fly_route.py "$ROUTE" "$@" 2>&1 | tee "$LOG"

ln -sf "flight_${TS}.log" out/logs/latest.log
echo
echo "== bitti. asistana: 'bitti' de -- o out/logs/latest.log + out/state_log.csv okur =="
