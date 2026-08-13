#!/usr/bin/env python3
"""PX4 parametresini MAVLink üzerinden ayarlar ve GERİ OKUYUP doğrular.

Neden: pxh> konsolu optik-akış sensörünün "Number of good matches" spam'iyle
boğulduğunda param set/show çıktısı görünmüyor. Bu betik ayarı temiz bir
terminalden yapıp değeri geri okur -- konsola hiç dokunmadan.

    # sim çalışırken, fly_route KOŞMUYORKEN (14540 portu boş olmalı)
    .venv/bin/python scripts/set_param.py EKF2_OF_CTRL 0
    .venv/bin/python scripts/set_param.py EKF2_OF_CTRL      # sadece oku

NOT: EKF2_OF_CTRL gibi bazı EKF2 parametreleri kestirici BAŞLANGICINDA okunur;
canlı set anında devreye girmeyebilir. Ayar SITL param dosyasına yazılır ve
KALICIDIR -> uygulanması için SİM'İ YENİDEN BAŞLAT (yeni EKF2 init değeri okur).
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="parametre adı (ör. EKF2_OF_CTRL)")
    ap.add_argument("value", nargs="?", default=None,
                    help="yeni değer (verilmezse sadece okur)")
    ap.add_argument("--url", default="udpin:0.0.0.0:14540",
                    help="PX4 MAVLink (fly_route ile aynı; o koşmuyorken boş)")
    ap.add_argument("--int", action="store_true",
                    help="INT32 olarak yaz (EKF2_*_CTRL bit maskeleri int)")
    args = ap.parse_args()

    name = args.name.encode() if isinstance(args.name, str) else args.name
    print(f"bağlanılıyor: {args.url} ...")
    m = mavutil.mavlink_connection(args.url, source_system=1, source_component=190)
    if not m.wait_heartbeat(timeout=15):
        print("HATA: heartbeat yok. Sim çalışıyor mu? 14540 boş mu "
              "(fly_route kapalı mı)?", file=sys.stderr)
        return 1
    print(f"  bağlandı: sistem {m.target_system}")

    def read(timeout=5.0):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name, -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg and msg.param_id.strip("\x00") == args.name:
                return msg
        return None

    if args.value is not None:
        # EKF2_*_CTRL bit maskeleri INT32; değeri int gibi yolla.
        ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32
                 if (args.int or "_CTRL" in args.name) else
                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        val = float(int(float(args.value))) if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32 \
            else float(args.value)
        m.mav.param_set_send(m.target_system, m.target_component, name, val, ptype)
        time.sleep(0.3)

    msg = read()
    if msg is None:
        print(f"HATA: {args.name} okunamadı.", file=sys.stderr)
        return 1
    print(f"{args.name} = {msg.param_value:g}"
          + (f"   (istenen: {args.value})" if args.value is not None else ""))
    if args.value is not None and abs(msg.param_value - float(args.value)) > 1e-6:
        print("UYARI: değer istenene EŞİT DEĞİL.", file=sys.stderr)
        return 2
    if args.value is not None:
        print("\nYAZILDI (kalıcı). Devreye girmesi için SİM'İ YENİDEN BAŞLAT.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
