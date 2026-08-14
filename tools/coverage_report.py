#!/usr/bin/env python3
"""Kapsama raporu: taranan kutuları (scan_boxes -> scans.json) ground_truth ile
karşılaştırır ve depoyu HİYERARŞİK olarak değerlendirir:

    Depo -> Koridor -> Raf Yüzü -> Raf Seviyesi -> Göz -> Envanter

Her düğüm için durum: tarandı / kısmen tarandı / taranmadı.

    .venv/bin/python tools/coverage_report.py
    .venv/bin/python tools/coverage_report.py --missed   # kaçanları tek tek
    .venv/bin/python tools/coverage_report.py --html out/coverage.html

Çıktılar: terminal raporu + out/coverage_report.json (+ isteğe bağlı HTML).

Bir GEÇİŞ = (raf yüzü, seviye) ikilisi; tam kapsama 4 yüz x 3 seviye = 12 geçiş.
Eşleştirme anahtarı kutu QR yükü: tip içinde benzersiz olan tek şey o (barkod
yükleri benzersiz değil, bu yüzden kapsama QR üstünden sayılır).

Uçuş/ROS gerektirmez; JSON'ları okur. Koridor eşlemesi depo yerleşiminden
(config/warehouse.yaml) türetilir -- elle yazılmış "A,B -> 1" tablosu yoktu,
yerleşim değişince sessizce yanlış olurdu.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent

FULL, PARTIAL, EMPTY = "✅ tam", "🟡 kısmi", "⬜ boş"
STATUS = {"full": "tarandı", "partial": "kısmen tarandı", "empty": "taranmadı"}


def status_of(scanned: int, total: int) -> str:
    if total == 0 or scanned == 0:
        return "empty"
    return "full" if scanned == total else "partial"


def row_to_aisle(cfg: dict) -> dict[str, int]:
    """Raf yüzü -> koridor. Yüzün baktığı yöne en yakın koridor merkezinden
    türetilir; sabit tablo yerleşim değişince sessizce yanlış olurdu."""
    r = cfg["racking"]
    out = {}
    for row in r["rows"]:
        face_y = row["y0"] + r["depth"] if row["facing"] > 0 else row["y0"]
        out[row["id"]] = min(r["aisles"],
                             key=lambda a: abs(a["y_center"] - face_y))["id"]
    return out


def build_tree(truth: list, scanned: set, aisle_of: dict) -> dict:
    """Depo -> koridor -> yüz -> seviye -> göz ağacı; her düğümde sayaç."""
    tree: dict = {"total": 0, "scanned": 0, "aisles": {}}
    for c in truth:
        a = aisle_of.get(c["row"], 0)
        hit = c["payload"] in scanned
        node_a = tree["aisles"].setdefault(a, {"total": 0, "scanned": 0, "faces": {}})
        node_f = node_a["faces"].setdefault(
            c["row"], {"total": 0, "scanned": 0, "levels": {}})
        node_l = node_f["levels"].setdefault(
            c["level"], {"total": 0, "scanned": 0, "bays": {}})
        node_b = node_l["bays"].setdefault(c["bay"], {"total": 0, "scanned": 0})
        for n in (tree, node_a, node_f, node_l, node_b):
            n["total"] += 1
            n["scanned"] += hit
    return tree


def annotate(node: dict) -> dict:
    """Ağaca durum etiketleri ve yüzde ekler (yerinde, özyinelemeli)."""
    node["status"] = status_of(node["scanned"], node["total"])
    node["pct"] = round(100 * node["scanned"] / node["total"], 1) if node["total"] else 0.0
    for key in ("aisles", "faces", "levels", "bays"):
        for child in node.get(key, {}).values():
            annotate(child)
    return node


def write_html(tree: dict, path: Path, hit: int, total: int) -> None:
    """Basit görsel rapor: yüz x seviye ızgarası, göz kırılımıyla.

    Tek dosya, dış bağımlılık yok -- tarayıcıda çift tıklayınca açılır.
    """
    color = {"full": "#2e7d32", "partial": "#f9a825", "empty": "#c62828"}
    p = ['<meta charset="utf-8"><title>Kapsama Raporu</title>',
         '<style>body{font-family:system-ui,sans-serif;margin:2rem;background:#fafafa;color:#222}'
         'h1{font-size:1.4rem}h2{font-size:1.05rem;margin-top:1.6rem}'
         'table{border-collapse:collapse;margin:.5rem 0}'
         'td,th{border:1px solid #ddd;padding:.35rem .6rem;font-size:.9rem;text-align:center}'
         'th{background:#f0f0f0}.b{color:#fff;border-radius:3px;padding:.15rem .4rem;font-size:.8rem}'
         '.sum{font-size:1.1rem;margin:.6rem 0}</style>',
         f'<h1>Depo Kapsama Raporu</h1>',
         f'<p class="sum"><b>{hit}/{total}</b> kutu tarandı '
         f'(%{100*hit/total:.1f})</p>' if total else '']
    for aid, a in sorted(tree["aisles"].items()):
        p.append(f'<h2>Koridor {aid} — {a["scanned"]}/{a["total"]} '
                 f'(%{a["pct"]})</h2>')
        for fid, f in sorted(a["faces"].items()):
            levels = sorted(f["levels"])
            bays = sorted({b for l in f["levels"].values() for b in l["bays"]})
            p.append(f'<b>Raf yüzü {fid}</b> — {f["scanned"]}/{f["total"]} '
                     f'<span class="b" style="background:{color[f["status"]]}">'
                     f'{STATUS[f["status"]]}</span>')
            p.append('<table><tr><th>seviye</th>'
                     + "".join(f"<th>göz {b}</th>" for b in bays)
                     + '<th>seviye toplam</th></tr>')
            for lid in levels:
                l = f["levels"][lid]
                cells = []
                for b in bays:
                    n = l["bays"].get(b)
                    if n is None:
                        cells.append("<td>-</td>")
                    else:
                        cells.append(
                            f'<td style="background:{color[n["status"]]};color:#fff">'
                            f'{n["scanned"]}/{n["total"]}</td>')
                cells.append(f'<td>{l["scanned"]}/{l["total"]} '
                             f'({STATUS[l["status"]]})</td>')
                p.append(f'<tr><th>L{lid}</th>' + "".join(cells) + '</tr>')
            p.append('</table>')
    path.write_text("\n".join(p))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--scans", type=Path,
                    default=PROJECT_ROOT / "out" / "scans" / "scans.json")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--json", type=Path,
                    default=PROJECT_ROOT / "out" / "coverage_report.json")
    ap.add_argument("--html", type=Path,
                    help="basit görsel raporu bu dosyaya yaz (örn. out/coverage.html)")
    ap.add_argument("--missed", action="store_true",
                    help="kaçan (taranmamış) kutuları tek tek listele")
    args = ap.parse_args()

    codes = json.loads(args.ground_truth.read_text())["codes"]
    truth = [c for c in codes if c["type"] == "box_qr"]
    if not truth:
        print("ground truth'ta box_qr yok.")
        return 1
    cfg = yaml.safe_load(args.config.read_text())
    aisle_of = row_to_aisle(cfg)

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
    tree = annotate(build_tree(truth, scanned, aisle_of))

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
    print("koridor: " + " ; ".join(f"{r} -> {aisle_of[r]}" for r in rows) + "\n")

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

    # ---- hiyerarşik durum
    print("\nhiyerarşik durum:")
    print(f"  Depo                    {tree['scanned']:3d}/{tree['total']:<3d} "
          f"{STATUS[tree['status']]}")
    for aid, a in sorted(tree["aisles"].items()):
        print(f"    Koridor {aid}             {a['scanned']:3d}/{a['total']:<3d} "
              f"{STATUS[a['status']]}")
        for fid, f in sorted(a["faces"].items()):
            print(f"      Raf yüzü {fid}          {f['scanned']:3d}/{f['total']:<3d} "
                  f"{STATUS[f['status']]}")
            for lid, l in sorted(f["levels"].items()):
                bad = [f"göz {b}" for b, n in sorted(l["bays"].items())
                       if n["status"] != "full"]
                note = ("  eksik: " + ", ".join(bad)) if bad else ""
                print(f"        Seviye {lid}        {l['scanned']:3d}/{l['total']:<3d} "
                      f"{STATUS[l['status']]}{note}")

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
    missed = [c for c in truth if c["payload"] not in scanned]
    if args.missed:
        print(f"\nkaçan kutular ({len(missed)}):")
        for c in sorted(missed, key=lambda c: (c["row"], c["level"], c["bay"])):
            print(f"  {c['row']}-{c['bay']:02d}-L{c['level']}  {c['payload']}")

    # ---- dosyalar
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "source": str(args.scans),
        "total_boxes": total,
        "scanned_boxes": hit,
        "coverage_pct": round(100 * hit / total, 1),
        "passes": {"full": full, "partial": partial, "empty": empty},
        "unknown_payloads": sorted(unknown),
        "missed": [{"row": c["row"], "bay": c["bay"], "level": c["level"],
                    "payload": c["payload"]} for c in missed],
        "tree": tree,
    }, indent=2, ensure_ascii=False))
    print(f"\nrapor: {args.json}")
    if args.html:
        write_html(tree, args.html, hit, total)
        print(f"görsel: {args.html}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
