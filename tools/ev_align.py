#!/usr/bin/env python3
"""EV besleme logunu (out/logs/ev_feed.csv) gerçek poz iziyle
(out/state_log.csv) DUVAR SAATİNE göre hizalar.

Amaç: "L1 kaçışı anında raf tag'i besleniyor muydu, besleniyorsa doğru mu?"
sorusunu KESİN cevaplamak. İki mekanizmayı ayırır:
  (H1) EV BOŞLUĞU  -> kaçış anında hiç fix yok (tag görülmüyor/reddediliyor)
                      => sorun tespit/görüş; çözüm: bilinen-geometri raf çözücü.
  (H2) EV KAYIYOR   -> fix var ama kestirim gerçekten uzak (tag yanlış poz)
                      => çözüm: kapı/çözücü kalitesi.
  (H3) EV SAĞLAM ama araç yine de sapıyor -> EV besleniyor, kestirim doğru,
                      ama EKF akış çöpünü tercih ediyor => çözüm: EKF ağırlık/hız.

    .venv/bin/python tools/ev_align.py            # tüm uçuş, 2 s'de bir özet
    .venv/bin/python tools/ev_align.py --from 100 --to 125   # kaçış penceresi
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load(path: Path) -> list[dict]:
    with path.open() as f:
        return [{k: v for k, v in row.items()} for row in csv.DictReader(f)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=Path, default=ROOT / "out" / "state_log.csv")
    ap.add_argument("--ev", type=Path, default=ROOT / "out" / "logs" / "ev_feed.csv")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0)
    ap.add_argument("--to", dest="t_to", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=2.0, help="özet aralığı (s)")
    args = ap.parse_args()

    st = load(args.state)
    if not st or "wall_ms" not in st[0]:
        print("state_log'da wall_ms yok -- log_state.py'yi güncel sürümle koş.")
        return 1
    ev = load(args.ev) if args.ev.exists() else []
    for r in st:
        r["t_s"] = float(r["t_s"]); r["wall_ms"] = int(r["wall_ms"])
        for k in ("x", "y", "z"):
            r[k] = float(r[k])
    for e in ev:
        e["wall_ms"] = int(e["wall_ms"])
        for k in ("x", "y", "z"):
            e[k] = float(e[k])

    # her EV fix'i wall_ms'e göre sıralı; en yakın truth satırını bul (hata için).
    ev.sort(key=lambda e: e["wall_ms"])
    st.sort(key=lambda r: r["wall_ms"])

    def truth_at(wall_ms: int):
        # en yakın state satırı (basit lineer; loglar küçük)
        best = min(st, key=lambda r: abs(r["wall_ms"] - wall_ms))
        return best if abs(best["wall_ms"] - wall_ms) < 500 else None

    n_rack = sum(1 for e in ev if e["src"] == "rack")
    n_floor = sum(1 for e in ev if e["src"] == "floor")
    print(f"state: {len(st)} satır   EV fix: {len(ev)}  (rack {n_rack}, floor {n_floor})")
    if not ev:
        print("EV log BOŞ -> localizer hiç fix beslemedi (ya da --ev-log kapalıydı).")

    # EV fix'lerini truth zaman eksenine (t_s) taşı: en yakın state satırının t_s'i.
    for e in ev:
        tr = truth_at(e["wall_ms"])
        e["t_s"] = tr["t_s"] if tr else None
        e["err_cm"] = (100 * math.hypot(e["x"] - tr["x"], e["y"] - tr["y"])
                       if tr else None)

    print(f"\n{'t_s':>6} {'truth x':>8} {'y':>6} {'z':>5} | "
          f"{'sonEV':>6} {'boşluk':>6} {'kaynak':>6} {'EVhata':>7}")
    print("-" * 60)
    t = args.t_from
    evs_in = [e for e in ev if e["t_s"] is not None]
    while t <= min(args.t_to, st[-1]["t_s"]):
        # bu ana en yakın truth
        tr = min(st, key=lambda r: abs(r["t_s"] - t))
        # bu ana kadarki son EV fix
        prior = [e for e in evs_in if e["t_s"] <= t]
        if prior:
            last = max(prior, key=lambda e: e["t_s"])
            gap = t - last["t_s"]
            src = last["src"]
            err = last["err_cm"]
            errs = f"{err:5.1f}cm" if err is not None else "   -"
            gaps = f"{gap:4.1f}s"
            # boşluk büyükse işaretle
            flag = "  <-- BOŞLUK" if gap > 1.0 else ("  <-- KAYMA" if (err or 0) > 40 else "")
        else:
            src, gaps, errs, flag = "—", "  —", "   -", ""
        print(f"{t:6.1f} {tr['x']:8.2f} {tr['y']:6.2f} {tr['z']:5.2f} | "
              f"{src:>6} {gaps:>6} {'':>0}{errs:>7}{flag}")
        t += args.step
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
