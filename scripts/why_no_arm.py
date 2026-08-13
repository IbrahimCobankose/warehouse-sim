#!/usr/bin/env python3
"""PX4 neden arm'ı reddediyor? Arm dener, PX4'ün gerekçesini (STATUSTEXT +
COMMAND_ACK) temiz terminale basar. Reddedilirse araç havalanmaz (güvenli);
kabul edilirse HEMEN disarm eder.

    # sim çalışırken, fly_route KOŞMUYORKEN (14540 boş)
    .venv/bin/python scripts/why_no_arm.py

EKF2_OF_CTRL 0 (akış kapalı) sonrası "arm reddedildi" için: gerekçe genelde
hız/konum kestirim referansı eksikliğidir (GPS+akış+görüş-hızı hiçbiri yoksa
PX4 offboard konum kontrolüne izin vermez). Metni buraya bakıp doğrularız.
"""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

SEV = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN",
       5: "NOTICE", 6: "INFO", 7: "DEBUG"}


def main() -> int:
    url = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14540"
    print(f"bağlanılıyor: {url} ...")
    m = mavutil.mavlink_connection(url, source_system=1, source_component=191)
    if not m.wait_heartbeat(timeout=15):
        print("HATA: heartbeat yok (sim kapalı ya da 14540 dolu).", file=sys.stderr)
        return 1
    print(f"  bağlandı: sistem {m.target_system}\n")

    # Önce mevcut STATUSTEXT akışını 1.5 s dinle (prearm uyarıları periyodik gelir)
    print("--- arm ÖNCESİ mesajlar (1.5 s) ---")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg:
            print(f"  [{SEV.get(msg.severity, msg.severity)}] {msg.text}")

    # Arm komutu gönder
    print("\n--- ARM deneniyor ---")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    armed = False
    t0 = time.time()
    while time.time() - t0 < 4.0:
        msg = m.recv_match(type=["STATUSTEXT", "COMMAND_ACK"], blocking=True,
                           timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  [{SEV.get(msg.severity, msg.severity)}] {msg.text}")
        else:  # COMMAND_ACK
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                res = {0: "KABUL", 1: "GEÇİCİ_RED", 2: "DESTEKLENMEZ",
                       3: "BAŞARISIZ", 4: "İPTAL"}.get(msg.result, msg.result)
                print(f"  >>> ARM sonucu: {res} (result={msg.result})")
                armed = (msg.result == 0)

    if armed:
        print("\narm KABUL edildi -> güvenlik için hemen disarm ediliyor.")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 0, 0, 0, 0, 0, 0, 0)
    print("\nİpucu: 'no valid velocity'/'pos'/'estimator' geçen bir satır varsa "
          "hız referansı eksik -> görüş HIZI beslemesi (VISION_SPEED_ESTIMATE) gerek.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
