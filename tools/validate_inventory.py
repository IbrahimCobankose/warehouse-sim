#!/usr/bin/env python3
"""Envanter doğrulama: inventory.json'ı ground_truth ile karşılaştırır.

    .venv/bin/python tools/validate_inventory.py
    .venv/bin/python tools/validate_inventory.py --min-confidence 0.3
    .venv/bin/python tools/validate_inventory.py --list-missed

Uçuş/ROS gerektirmez, iki JSON okur. Çıktı: terminal raporu +
out/validation_report.json.

ANA KPI -- envanter doğruluğu: bir kaydın DOĞRU sayılması için (a) yükü ground
truth'ta bulunmalı, (b) raf yüzü, seviye ve göz ataması doğru olmalı. Yani
"doğru ürünü doğru lokasyona koyduk mu". Konum hatası ayrıca metre cinsinden
raporlanıyor ama doğruluğun tanımına girmiyor: 5 cm'lik hata ürünü yanlış göze
taşımıyorsa envanter kaydı doğrudur.

ÇİFT KAYIT (duplicate) iki ayrı şey olabilir, ikisi de kontrol ediliyor:
  kimlik  -- aynı ürün kimliği birden çok kayıtta (dedup kaçağı)
  uzamsal -- iki farklı kayıt neredeyse aynı noktada (aynı fiziksel kutuyu
             iki kez saymış olma ihtimali; bu dünyada kutular ~0.4 m aralıklı)

GROUND TRUTH burada ÖLÇÜM için okunuyor; kestirime hiçbir şekilde girmiyor
(proje kuralı). Kestirim tools/build_inventory.py'de yapılır ve ground truth'a
bakmaz.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics as st
from collections import Counter, defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# İki kaydın "aynı fiziksel kutu" sayılacağı mesafe. Bu dünyada bir gözdeki
# kutular ~0.4 m aralıklı, o yüzden 0.10 m güvenli: gerçek komşu kutuları
# çift kayıt sanmaz.
DUP_DIST_M = 0.10


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path,
                    default=PROJECT_ROOT / "out" / "inventory.json")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--out", type=Path,
                    default=PROJECT_ROOT / "out" / "validation_report.json")
    ap.add_argument("--min-confidence", type=float, default=0.0,
                    help="bu güvenin altındaki kayıtları rapor dışı bırak")
    ap.add_argument("--list-missed", action="store_true",
                    help="kaçan kutuları tek tek listele")
    ap.add_argument("--list-worst", type=int, default=0,
                    help="konum hatası en büyük N kaydı listele")
    args = ap.parse_args()

    if not args.inventory.exists():
        print(f"envanter yok: {args.inventory}\n"
              f"önce: .venv/bin/python tools/build_inventory.py")
        return 1

    inv_doc = json.loads(args.inventory.read_text())
    items = inv_doc["items"]
    n_all = len(items)
    if args.min_confidence > 0:
        items = [i for i in items if i["confidence"] >= args.min_confidence]

    truth = {c["payload"]: c for c in
             json.loads(args.ground_truth.read_text())["codes"]
             if c["type"] == "box_qr"}
    total = len(truth)

    # ---------------------------------------------------------------- eşleşme
    matched, wrong_id, unreadable = [], [], []
    for it in items:
        if not it.get("readable", True):
            # Yükü çözülemeyen kayıt: hat onu "bilinmeyen ürün" olarak taşıyor,
            # sessizce atmıyor. Doğruluk paydasına giriyor ama doğru sayılmıyor.
            unreadable.append(it)
        elif it["qr"] in truth:
            matched.append(it)
        else:
            # Ground truth'ta olmayan yük: yanlış çözüm ya da bozuk doku.
            wrong_id.append(it)

    errs, wrong_shelf, wrong_level, wrong_bay, correct = [], [], [], [], 0
    for it in matched:
        t = truth[it["qr"]]
        tx, ty, tz = t["label_pose_xyzrpy"][:3]
        errs.append(math.dist((it["estimated_x"], it["estimated_y"],
                               it["estimated_z"]), (tx, ty, tz)))
        ok = True
        if it["shelf"] != t["row"]:
            wrong_shelf.append(it); ok = False
        if it["level"] != t["level"]:
            wrong_level.append(it); ok = False
        if it.get("bay") != t["bay"]:
            wrong_bay.append(it); ok = False
        correct += ok

    scanned = {it["qr"] for it in matched}
    missed = [c for c in truth.values() if c["payload"] not in scanned]

    # ------------------------------------------------------------- çift kayıt
    by_id = Counter(it["product_id"] for it in items if it.get("product_id"))
    dup_id = [k for k, v in by_id.items() if v > 1]

    dup_spatial = []
    pts = [(it, (it["estimated_x"], it["estimated_y"], it["estimated_z"]))
           for it in items]
    # Kutu sayısı birkaç yüz; kaba karşılaştırma yeterli, uzamsal indeks gereksiz.
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            if math.dist(pts[i][1], pts[j][1]) < DUP_DIST_M:
                dup_spatial.append([pts[i][0]["qr"], pts[j][0]["qr"]])

    # ------------------------------------------------------------------ rapor
    e = sorted(errs)
    denom = len(items) or 1
    accuracy = 100.0 * correct / denom
    pos = {}
    if e:
        pos = {
            "median_m": round(st.median(e), 3),
            "mean_m": round(st.mean(e), 3),
            "p95_m": round(e[int(0.95 * (len(e) - 1))], 3),
            "max_m": round(e[-1], 3),
            "within_10cm_pct": round(100 * sum(x <= 0.10 for x in e) / len(e), 1),
            "within_25cm_pct": round(100 * sum(x <= 0.25 for x in e) / len(e), 1),
        }

    report = {
        "inventory": str(args.inventory),
        "min_confidence": args.min_confidence,
        "records": {"total_in_file": n_all, "evaluated": len(items)},
        "detected": len(matched),
        "total_ground_truth": total,
        "detection_rate_pct": round(100 * len(matched) / total, 1) if total else 0,
        "missed": len(missed),
        "duplicate_id": len(dup_id),
        "duplicate_spatial": len(dup_spatial),
        "wrong_id": len(wrong_id),
        "unreadable": len(unreadable),
        "wrong_shelf": len(wrong_shelf),
        "wrong_level": len(wrong_level),
        "wrong_bay": len(wrong_bay),
        "correct_records": correct,
        "inventory_accuracy_pct": round(accuracy, 1),
        "position_error": pos,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))

    print("ENVANTER DOĞRULAMA")
    print(f"kaynak: {args.inventory}")
    if args.min_confidence > 0:
        print(f"güven eşiği: {args.min_confidence}  "
              f"({len(items)}/{n_all} kayıt değerlendiriliyor)")
    print()
    print(f"  tespit / toplam    : {len(matched)} / {total}  "
          f"(%{report['detection_rate_pct']})")
    print(f"  kaçan envanter     : {len(missed)}")
    print(f"  çift kayıt (kimlik): {len(dup_id)}")
    print(f"  çift kayıt (uzamsal): {len(dup_spatial)}")
    print(f"  yanlış ID          : {len(wrong_id)}")
    print(f"  çözülemeyen yük    : {len(unreadable)}")
    print(f"  yanlış raf         : {len(wrong_shelf)}")
    print(f"  yanlış seviye      : {len(wrong_level)}")
    print(f"  yanlış göz         : {len(wrong_bay)}")
    if pos:
        print(f"\n  konum hatası       : medyan {pos['median_m']:.3f} m   "
              f"P95 {pos['p95_m']:.3f} m   maks {pos['max_m']:.3f} m")
        print(f"                       %{pos['within_10cm_pct']:.1f} <=10 cm   "
              f"%{pos['within_25cm_pct']:.1f} <=25 cm")
    verdict = "GEÇTİ" if accuracy >= 95.0 else "KALDI"
    print(f"\n  ENVANTER DOĞRULUĞU : %{accuracy:.1f}  "
          f"({correct}/{len(items)} kayıt)   hedef >=%95 -> {verdict}")
    print(f"\nrapor: {args.out}")

    if args.list_missed and missed:
        print(f"\nkaçan kutular ({len(missed)}):")
        for c in sorted(missed, key=lambda c: (c["row"], c["level"], c["bay"])):
            print(f"  {c['row']}-{c['bay']:02d}-L{c['level']}  {c['payload']}")
    if args.list_worst and matched:
        worst = sorted(zip(errs, matched), key=lambda p: -p[0])[:args.list_worst]
        print(f"\nkonum hatası en büyük {len(worst)} kayıt:")
        for err, it in worst:
            print(f"  {err:5.2f} m  {it['shelf']}-{it.get('bay', 0):02d}-"
                  f"L{it['level']}  {it['product_id']:<10} "
                  f"conf={it['confidence']:.2f} n={it['n_readings']}")
    if dup_id:
        print(f"\nÇİFT KİMLİK: {', '.join(dup_id[:10])}")
    if wrong_id:
        print(f"\nGROUND TRUTH'TA OLMAYAN YÜK ({len(wrong_id)}):")
        for it in wrong_id[:10]:
            print(f"  {it['qr']!r}")

    return 0 if accuracy >= 95.0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
