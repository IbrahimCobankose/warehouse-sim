#!/usr/bin/env python3
"""Kapsama raporu: taranan kutuları (scan_boxes -> scans.json) ground_truth ile
karşılaştırır. Hangi raf yüzü / seviye / göz tarandı, hangisi kaçtı -- 6.
aşamanın "bitti" ölçütü ve rota genişletmesinin yol göstericisi.

Bir GEÇİŞ = (raf yüzü, seviye) ikilisi; tam kapsama 4 yüz x 3 seviye = 12 geçiş
(bkz. README). Eşleştirme anahtarı kutu QR yükü: tip içinde benzersiz olan tek
şey o (barkod yükleri benzersiz değil, bu yüzden kapsama QR üstünden sayılır).

    .venv/bin/python tools/coverage_report.py
    .venv/bin/python tools/coverage_report.py --missed   # kaçanları tek tek

Uçuş/ROS gerektirmez; iki JSON'u okur. scan_boxes'ı bir tur boyunca koşturup
(bkz. scripts/scan_boxes.py) ürettiği scans.json'u buraya verin.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

FULL, PARTIAL, EMPTY = "✅ tam", "🟡 kısmi", "⬜ boş"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--scans", type=Path,
                    default=PROJECT_ROOT / "out" / "scans" / "scans.json")
    ap.add_argument("--missed", action="store_true",
                    help="kaçan (taranmamış) kutuları tek tek listele")
    args = ap.parse_args()

    codes = json.loads(args.ground_truth.read_text())["codes"]
    truth = [c for c in codes if c["type"] == "box_qr"]
    if not truth:
        print("ground truth'ta box_qr yok.")
        return 1

    scanned: set[str] = set()
    n_records = 0
    if args.scans.exists():
        data = json.loads(args.scans.read_text())
        for r in data.get("boxes", []):
            n_records += 1
            # Yalnızca ground truth'ta eşleşen (known) yükler kapsamaya sayılır;
            # eşleşmeyenler aşağıda ayrı uyarı olarak raporlanır.
            if r.get("known"):
                scanned.add(r["payload"])
        gt_payloads = {c["payload"] for c in truth}
        unknown = {r["payload"] for r in data.get("boxes", [])
                   if not r.get("known") and r["payload"] not in gt_payloads}
    else:
        print(f"UYARI: scan dosyası yok ({args.scans}) -- kapsama %0 gösterilecek.\n")
        unknown = set()

    rows = sorted({c["row"] for c in truth})
    levels = sorted({c["level"] for c in truth})

    # (yüz, seviye) -> [toplam, taranan]
    cell = defaultdict(lambda: [0, 0])
    for c in truth:
        k = (c["row"], c["level"])
        cell[k][0] += 1
        if c["payload"] in scanned:
            cell[k][1] += 1

    total = len(truth)
    hit = len(scanned & {c["payload"] for c in truth})

    print("KAPSAMA RAPORU")
    print(f"kaynak: {args.scans}  ({n_records} kayıt, {hit} eşleşen kutu)")
    print(f"koridor: A,B -> 1 ; C,D -> 2\n")

    # ---- geçiş matrisi (taranan / toplam)
    print("geçiş matrisi  (taranan / toplam kutu):")
    header = "        " + "   ".join(f"L{l}".center(7) for l in levels) + "    yüz"
    print(header)
    for r in rows:
        parts = []
        rtot = rsc = 0
        for l in levels:
            t, s = cell[(r, l)]
            rtot += t; rsc += s
            parts.append(f"{s:2d}/{t:<2d}".center(7))
        print(f"  {r}    " + "   ".join(parts) + f"    {rsc:2d}/{rtot}")
    print(f"\n  TOPLAM: {hit}/{total} kutu  (%{100*hit/total:.0f})")

    # ---- geçiş durumu (12 geçiş)
    full, partial, empty = [], [], []
    for r in rows:
        for l in levels:
            t, s = cell[(r, l)]
            tag = f"{r}-L{l}"
            if t == 0:
                continue
            if s == t:
                full.append(tag)
            elif s == 0:
                empty.append(tag)
            else:
                partial.append(f"{tag} ({s}/{t})")
    print(f"\ngeçiş durumu ({len(full)+len(partial)+len(empty)} geçiş):")
    print(f"  {FULL}   : {', '.join(full) or '-'}")
    print(f"  {PARTIAL} : {', '.join(partial) or '-'}")
    print(f"  {EMPTY}   : {', '.join(empty) or '-'}")

    if unknown:
        print(f"\nUYARI: ground truth'ta olmayan {len(unknown)} yük tarandı "
              f"(yanlış render / aynalanmış doku olabilir):")
        for p in sorted(unknown)[:10]:
            print(f"  {p!r}")

    # ---- kaçanlar
    if args.missed:
        missed = [c for c in truth if c["payload"] not in scanned]
        print(f"\nkaçan kutular ({len(missed)}):")
        for c in sorted(missed, key=lambda c: (c["row"], c["level"], c["bay"])):
            print(f"  {c['row']}-{c['bay']:02d}-L{c['level']}  {c['payload']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
