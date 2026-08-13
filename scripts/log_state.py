#!/usr/bin/env python3
"""Aracın anlık konumunu bir DOSYAYA yazan bağımsız kaydedici.

Uçuşu HİÇ etkilemez: yalnızca Gazebo'nun gerçek poz akışını
(`/world/<world>/dynamic_pose/info`) `gz topic -e` alt süreciyle okur, PX4'e
ya da MAVLink'e dokunmaz. `measure_drift.py`'deki `TruthFeed` ile aynı kaynak.
Amaç: uçuş sırasında/sonrasında konum-zaman izini `out/state_log.csv`'ye
dökmek ki analiz için paylaşılabilsin.

    # ayrı bir terminalde, sim çalışırken (fly_route'dan bağımsız, aynı anda)
    .venv/bin/python scripts/log_state.py
    # dosya: out/state_log.csv  (t_s, x, y, z, yaw_deg)  -- Ctrl-C ile biter

GROUND TRUTH ASLA kontrole girmez (proje kuralı); burada sadece "araç gerçekte
neredeydi" sorusunu kaydetmek için okunuyor. Sütunlar dünya (Gazebo ENU)
koordinatında: x=Doğu, y=Kuzey, z=yukarı; yaw_deg = dünya +X'ten CCW.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_drift import TruthFeed                              # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--model", default="warehouse_scout_0",
                    help="gz model adı (PX4 gz_bridge log'unda 'model: ...')")
    ap.add_argument("--out", type=Path,
                    default=PROJECT_ROOT / "out" / "state_log.csv")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="saniyedeki kayıt sayısı (Hz)")
    ap.add_argument("--quiet", action="store_true",
                    help="canlı satır basma, yalnız dosyaya yaz")
    args = ap.parse_args()

    truth = TruthFeed(args.world, args.model)
    print(f"gerçek poz bekleniyor: /world/{args.world}/dynamic_pose/info "
          f"(model {args.model})...", file=sys.stderr)
    # ilk örnek gelene kadar bekle (sim kapalıysa burada takılır -> Ctrl-C)
    t_wait = time.time()
    while truth.pose is None:
        if time.time() - t_wait > 30:
            print("30 s içinde poz gelmedi. Sim çalışıyor mu? Model adı doğru mu "
                  "(--model)?", file=sys.stderr)
            truth.stop()
            return 1
        time.sleep(0.05)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    t0 = time.time()
    n = 0
    print(f"kaydediliyor -> {args.out}  ({args.rate:.0f} Hz, Ctrl-C ile bitir)",
          file=sys.stderr)
    try:
        with args.out.open("w") as f:
            # wall_ms: duvar-saati (Unix epoch, ms). EV besleme loguyla
            # (out/logs/ev_feed.csv, aynı saat) hizalamak için -- "L1 kaçışı
            # anında raf tag'i besleniyor muydu?" sorusunu cevaplar.
            f.write("t_s,wall_ms,x,y,z,yaw_deg\n")
            while True:
                loop_t = time.time()
                p, y = truth.pose, truth.yaw
                if p is not None:
                    t = loop_t - t0
                    yaw_deg = (y * 180.0 / 3.141592653589793
                               if y is not None else float("nan"))
                    f.write(f"{t:.3f},{int(loop_t*1000)},"
                            f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},"
                            f"{yaw_deg:.2f}\n")
                    f.flush()                     # canlı okunabilsin diye her satır
                    n += 1
                    if not args.quiet and n % max(1, int(args.rate)) == 0:
                        print(f"  t={t:6.1f}s  x={p[0]:+7.3f} y={p[1]:+7.3f} "
                              f"z={p[2]:5.3f}  yaw={yaw_deg:+6.1f}°  ({n} kayıt)",
                              file=sys.stderr)
                dt = period - (time.time() - loop_t)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        truth.stop()
    print(f"\n{n} kayıt yazıldı -> {args.out}  ({time.time() - t0:.1f} s)",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
