#!/usr/bin/env python3
"""PX4 EKF sağlık bayraklarını + innovation oranlarını okur (ESTIMATOR_STATUS).

"Preflight Fail: heading estimate not stable" / arm GEÇİCİ_RED'in KESİN sebebini
gösterir: hangi kestirim sağlıksız (attitude/hız/konum) ve yön/mag innovation
oranı ne. Oran > 1 = o ölçüm tutarlılık testini GEÇEMİYOR.

    # sim çalışırken, fly_route KOŞMUYORKEN
    .venv/bin/python scripts/est_status.py                 # 14540
    .venv/bin/python scripts/est_status.py udpin:0.0.0.0:14550
"""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

# ESTIMATOR_STATUS_FLAGS bit adları (MAVLink)
FLAGS = [
    ("ATTITUDE", 1), ("VELOCITY_HORIZ", 2), ("VELOCITY_VERT", 4),
    ("POS_HORIZ_REL", 8), ("POS_HORIZ_ABS", 16), ("POS_VERT_ABS", 32),
    ("POS_VERT_AGL", 64), ("CONST_POS_MODE", 128), ("PRED_POS_HORIZ_REL", 256),
    ("PRED_POS_HORIZ_ABS", 512), ("GPS_GLITCH", 1024), ("ACCEL_ERROR", 2048),
]


def main() -> int:
    url = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14540"
    print(f"bağlanılıyor: {url} ...")
    m = mavutil.mavlink_connection(url, source_system=1, source_component=192)
    if not m.wait_heartbeat(timeout=15):
        print("HATA: heartbeat yok.", file=sys.stderr)
        return 1
    print(f"  bağlandı: sistem {m.target_system}")

    # ESTIMATOR_STATUS (id 230) akışını iste
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            230, 200000, 0, 0, 0, 0, 0)   # 200000us = 5 Hz

    print("\nESTIMATOR_STATUS bekleniyor (~5 s)...\n")
    t0 = time.time()
    last = None
    while time.time() - t0 < 6.0:
        msg = m.recv_match(type="ESTIMATOR_STATUS", blocking=True, timeout=1.0)
        if msg:
            last = msg
    if last is None:
        print("ESTIMATOR_STATUS gelmedi (mesaj akmıyor olabilir).", file=sys.stderr)
        return 1

    fl = last.flags
    on = [n for n, b in FLAGS if fl & b]
    off = [n for n, b in FLAGS if not (fl & b)]
    print("SAĞLIKLI (bayrak açık):", ", ".join(on) or "—")
    print("EKSİK  (bayrak KAPALI):", ", ".join(n for n in off
          if n not in ("CONST_POS_MODE", "GPS_GLITCH", "ACCEL_ERROR")) or "—")
    print()
    print("innovation test oranları (>1.0 = TEST GEÇMİYOR):")
    print(f"  hız        vel_ratio  = {last.vel_ratio:.2f}")
    print(f"  yatay konum pos_ratio  = {last.pos_horiz_ratio:.2f}")
    print(f"  dikey konum hgt(vert)  = {last.pos_vert_ratio:.2f}")
    print(f"  MANYETİK   mag_ratio  = {last.mag_ratio:.2f}   <- yön (heading) buradan")
    print(f"  yükseklik   hagl_ratio = {last.hagl_ratio:.2f}")
    print()
    if not (fl & 1):
        print(">> ATTITUDE bayrağı KAPALI: yön/tavır oturmamış -> arm bloklanır.")
    if last.mag_ratio > 1.0:
        print(">> mag_ratio > 1: manyetik yön reddediliyor (mag yok/bozuk).")
    if not (fl & 2):
        print(">> VELOCITY_HORIZ KAPALI: yatay hız gözlenemiyor (akış yerde zayıf).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
