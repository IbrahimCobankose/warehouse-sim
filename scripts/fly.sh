#!/usr/bin/env bash
# 2/3 -- UÇURUR. Ortam scripts/sim.sh ile ayakta olmalı.
#
#   ./scripts/fly.sh                      # tam_tur_geo, ~13 dk
#   ./scripts/fly.sh --route koridor1_A_seviye2
#   ./scripts/fly.sh --no-check           # hazırlık denetimini atla
#   ./scripts/fly.sh -- --abort-offset 1.3   # -- sonrası fly_route'a geçer
#
# Loglama BU BETİKTE değil, flight_logged.sh'de: out/logs/flight_<tarih>.log
# (+ latest.log) ve arka planda log_state -> out/state_log.csv.
# Uçuş bitince: tarama sekmesinde Ctrl-C, sonra ./scripts/analyze.sh

set -uo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
export PATH="$PROJECT_DIR/scripts/bin:$PATH"
# shellcheck source=scripts/lib_wait.sh
source "$PROJECT_DIR/scripts/lib_wait.sh"

ROUTE="tam_tur_geo"; CHECK=1; RUN=""
while [ $# -gt 0 ]; do
    case "$1" in
        --route)    ROUTE="$2"; shift ;;
        --run)      RUN="$2"; shift ;;      # yalnız hatırlatma metni için
        --no-check) CHECK=0 ;;
        --)         shift; break ;;
        -h|--help)  sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "Bilinmeyen argüman: $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done

# Hazırlık denetimi KISA tutuluyor (20 s): burada beklemek değil, ortamın
# gerçekten ayakta olduğunu doğrulamak isteniyor. Ayakta değilse uçuşu
# başlatmak sessiz bir başarısızlık olurdu.
if [ "$CHECK" = 1 ]; then
    wait_for_sim 20 || { echo "Önce: ./scripts/sim.sh" >&2; exit 1; }
    wait_for_localize 20 || { echo "Önce: ./scripts/sim.sh (lokalizasyon sekmesi)" >&2; exit 1; }
fi

echo
echo "== uçuş: $ROUTE  (~13 dk, durdurmak: Ctrl-C) =="
bash scripts/flight_logged.sh "$ROUTE" --routes config/route_gen.yaml "$@"
rc=$?

cat <<NOTE

Uçuş bitti (çıkış $rc). Sırada:
  1) TARAMA sekmesinde Ctrl-C  -- özet ve son yazımlar orada basılır
  2) ./scripts/analyze.sh${RUN:+ --run $RUN}
     -> inventory.json/csv, validation_report.json, coverage.html, inventory_3d.html
NOTE
exit $rc
