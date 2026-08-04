#!/usr/bin/env python3
"""Simülasyondan kaydedilmiş kamera karelerindeki kodları çözer.

Bu, okunabilirlik zincirinin son halkası: gen_labels.py'nin bütçesi ve
check_readability.py'nin sentetik testi tahmindi; burada gerçek Gazebo
render'ından gelen kareler okunuyor. Çözülen yükler ground_truth.json ile
karşılaştırılıp doğru mu diye bakılıyor.

    .venv/bin/python tools/decode_frames.py out/frames/front
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from PIL import Image
from pyzbar import pyzbar

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", type=Path, help="kare PNG'lerini içeren dizin")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--limit", type=int, help="en fazla bu kadar kare işle")
    ap.add_argument("--verbose", action="store_true", help="kare kare göster")
    args = ap.parse_args()

    files = sorted(p for p in args.frames.iterdir()
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    if args.limit:
        files = files[:args.limit]
    if not files:
        print(f"{args.frames} içinde kare yok.")
        return 1

    truth = {}
    if args.ground_truth.exists():
        data = json.loads(args.ground_truth.read_text())
        truth = {c["payload"]: c for c in data["codes"]}

    found = Counter()
    frames_with_code = 0
    unknown = set()

    for f in files:
        try:
            results = pyzbar.decode(Image.open(f))
        except Exception as exc:                       # bozuk/yarım yazılmış kare
            if args.verbose:
                print(f"  {f.name}: okunamadı ({exc})")
            continue
        payloads = {r.data.decode("utf-8", "replace") for r in results}
        if payloads:
            frames_with_code += 1
        for p in payloads:
            found[p] += 1
            if truth and p not in truth:
                unknown.add(p)
        if args.verbose:
            print(f"  {f.name}: {sorted(payloads) if payloads else '-'}")

    print(f"kare              : {len(files)}")
    print(f"kod içeren kare   : {frames_with_code} "
          f"({100*frames_with_code/len(files):.0f}%)")
    print(f"benzersiz kod     : {len(found)}")

    if truth:
        by_type = Counter(truth[p]["type"] for p in found if p in truth)
        for t, n in sorted(by_type.items()):
            total = sum(1 for c in truth.values() if c["type"] == t)
            print(f"  {t:<14}: {n}/{total} okundu")
        if unknown:
            # Ground truth ile eşleşmeyen bir çözüm, kodun yanlış render
            # edildiğine (örn. aynalanmış doku) işaret eder -- sessizce
            # geçilmemeli.
            print(f"\nUYARI: ground truth'ta olmayan {len(unknown)} kod çözüldü:")
            for p in sorted(unknown)[:10]:
                print(f"  {p!r}")

    if found:
        print("\nen çok görülen kodlar:")
        for p, n in found.most_common(10):
            kind = truth.get(p, {}).get("type", "?")
            print(f"  {n:>4} kare  {kind:<14} {p}")
    else:
        print("\nHiç kod çözülemedi.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
