#!/usr/bin/env bash
# Uçuş sonrası dört adım, doğru sırayla (envanter -> doğrulama -> kapsama -> 3B).
#
#   ./scripts/analyze.sh                # varsayılan out/scans
#   ./scripts/analyze.sh --run run2     # start.sh --run run2 ile alınan koşu
#   ./scripts/analyze.sh --max-dt 0.5   # EV poz boşluğu geniş koşular için
#
# Sıra ÖNEMLİ: validate ve view, build_inventory'nin ürettiği inventory.json'u okur.

set -uo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
PY="$PROJECT_DIR/.venv/bin/python"

RUN=""; MAXDT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --run)     RUN="$2"; shift ;;
        --max-dt)  MAXDT="$2"; shift ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "Bilinmeyen argüman: $1" >&2; exit 2 ;;
    esac
    shift
done

S="${RUN:+_$RUN}"                      # koşu son eki
SCANS="out/scans$S"
INV="out/inventory$S.json"

[ -s "$SCANS/readings.jsonl" ] || { echo "[HATA] $SCANS/readings.jsonl yok." >&2; exit 1; }
[ -s "out/logs/ev_feed.csv" ]  || { echo "[HATA] out/logs/ev_feed.csv yok -- lokalizasyon koşmamış." >&2; exit 1; }

run() { echo; echo "== $* =="; "$@" || exit $?; }

run "$PY" tools/build_inventory.py --truth \
    --readings "$SCANS/readings.jsonl" \
    --out "$INV" --csv "out/inventory$S.csv" ${MAXDT:+--max-dt "$MAXDT"}

run "$PY" tools/validate_inventory.py --list-worst 5 \
    --inventory "$INV" --out "out/validation_report$S.json"

run "$PY" tools/coverage_report.py \
    --scans "$SCANS/scans.json" \
    --json "out/coverage_report$S.json" --html "out/coverage$S.html"

run "$PY" tools/view_inventory.py --truth \
    --inventory "$INV" --out "out/inventory_3d$S.html"

echo
echo "Raporlar:"
echo "  out/coverage$S.html"
echo "  out/inventory_3d$S.html"
