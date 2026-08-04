#!/usr/bin/env python3
"""Basit test uçuşu: kalkış, havada bekleme, iniş.

Amaç otonomi değil -- aracın uçtuğunu ve kameraların uçuş sırasında kullanılır
kare ürettiğini kanıtlamak. Otonom koridor taraması ayrı bir aşama.

Bekleme irtifası, alt kameranın altındaki zemin markörünü rahat çözeceği
şekilde seçilir (bkz. tools/gen_labels.py --budget).

    .venv/bin/python scripts/test_flight.py --altitude 2.0 --hover 12

Betik MAVLink üzerinden bir yer istasyonu gibi davranır (heartbeat yollar);
aksi halde PX4 "No connection to the ground control station" ile arm etmez.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

from pymavlink import mavutil

# Çıktı bir dosyaya yönlendirildiğinde tamponlanıp kaybolmasın: uçuş
# ilerlemesini canlı görmek bu betiğin bütün amacı.
sys.stdout.reconfigure(line_buffering=True)

HEARTBEAT_HZ = 2.0

#: Deponun dünya üzerindeki konumu -- gz/worlds/warehouse.sdf içindeki
#: spherical_coordinates ile aynı. GPS kapalıyken EKF origin'i bununla
#: sabitleniyor (bkz. Link.set_origin).
ORIGIN_LATLON = (41.015137, 28.979530)

_IS_TTY = sys.stdout.isatty()
_last_progress = 0.0


def progress(text: str, min_interval: float = 2.0) -> None:
    """İlerleme satırı. Terminalde aynı satırın üzerine yazar; çıktı bir
    dosyaya/boruya gidiyorsa seyrek aralıklarla normal satır basar (yoksa
    yüzlerce satırlık bir duvar oluşuyor)."""
    global _last_progress
    if _IS_TTY:
        print(f"  {text}", end="\r")
        return
    now = time.time()
    if now - _last_progress >= min_interval:
        _last_progress = now
        print(f"  {text}")


class Link:
    def __init__(self, url: str):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self._stop = threading.Event()
        self._hb = threading.Thread(target=self._beat, daemon=True)

    def _beat(self) -> None:
        while not self._stop.is_set():
            self.m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1.0 / HEARTBEAT_HZ)

    def start(self, timeout: float = 60.0) -> None:
        print("PX4 bekleniyor...")
        if not self.m.wait_heartbeat(timeout=timeout):
            raise SystemExit("HATA: PX4'ten heartbeat gelmedi. Simülasyon çalışıyor mu?")
        print(f"  bağlandı: sistem {self.m.target_system}, bileşen {self.m.target_component}")
        self._hb.start()
        time.sleep(2.0)     # PX4 bizi GCS olarak görsün

    def stop(self) -> None:
        self._stop.set()

    def cmd(self, command: int, *params: float) -> bool:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     command, 0, *p)
        ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        while ack and ack.command != command:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is None:
            print(f"  komut {command}: ACK yok")
            return False
        ok = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        if not ok:
            print(f"  komut {command} reddedildi (result={ack.result})")
        return ok

    def set_param(self, name: str, value: float, timeout: float = 5.0) -> bool:
        """Parametre yazar ve PX4'ün geri okuduğu değeri doğrular."""
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == name:
                return abs(msg.param_value - value) < 1e-3
        return False

    def set_origin(self, lat: float, lon: float, alt: float = 0.0) -> None:
        """EKF'in yerel origin'ini dünya üzerinde bir noktaya sabitler.

        GPS kapalıyken PX4'ün global konumu yok; AUTO modları (kalkış, iniş,
        Hold) ve home konumu buna bağlı olduğu için arm reddediliyor
        ("Arming denied: Resolve system health failures first").

        Bu, kestirime KONUM BİLGİSİ VERMEZ -- yalnızca zaten bilinen bir
        sabiti, deponun dünya üzerindeki yerini bildirir (world SDF'sindeki
        spherical_coordinates ile aynı değer). Gerçek bir kurulumda da depo
        koordinatı bilinir. Ground truth kuralı ihlal edilmiyor.

        Yükseklik 0 veriliyor: MAV_CMD_NAV_TAKEOFF'un param7'si deniz
        seviyesine göre yorumlanıyor, origin'i 40 m yaparsak 2.0 m'lik bir
        kalkış isteği "zaten daha yüksektesin" ile reddediliyor.
        """
        self.m.mav.set_gps_global_origin_send(
            self.m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt * 1000))

    def set_mode_hold(self) -> bool:
        """Hold (AUTO_LOITER). GPS'siz açılışta araç MANUAL'e düşüyor ve
        MANUAL, RC olmadığı için arm edilemiyor."""
        return self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                        4.0, 3.0)

    def state(self, timeout: float = 3.0):
        """(z_yukari_m, arm_durumu) döndürür."""
        pos = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
        hb = self.m.recv_match(type="HEARTBEAT", blocking=False)
        armed = None
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return (-pos.z if pos else None), armed


def wait_ready(link: Link, timeout: float = 120.0) -> None:
    """EKF'in yakınsamasını ve home konumunun kurulmasını bekler."""
    print("EKF / home bekleniyor...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = link.m.recv_match(type="EXTENDED_SYS_STATE", blocking=False)
        pos = link.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if pos is not None:
            print("  yerel konum kestirimi hazır")
            return
        time.sleep(0.2)
    raise SystemExit("HATA: EKF zamanında hazır olmadı.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--origin", nargs=2, type=float, metavar=("LAT", "LON"),
                    default=ORIGIN_LATLON,
                    help="deponun dünya üzerindeki yeri; GPS'siz uçuşta "
                         "AUTO modlarının çalışması için gerekli")
    ap.add_argument("--altitude", type=float, default=2.0, help="kalkış irtifası (m)")
    ap.add_argument("--hover", type=float, default=12.0, help="havada bekleme (s)")
    ap.add_argument("--yaw", type=float,
                    help="kalkıştan sonra bu yöne dön (derece, 0=+X/kuzey, "
                         "90=+Y). Ön kamera burnun baktığı yere bakar; koridor "
                         "X boyunca uzandığı için rafları görmek ancak ±90 ile "
                         "mümkün (-90 = A/C sırası, +90 = B/D sırası).")
    args = ap.parse_args()

    link = Link(args.url)
    link.start()
    wait_ready(link)

    # GPS kapalı: origin ve mod olmadan arm reddedilir.
    print(f"origin: {args.origin[0]:.6f}, {args.origin[1]:.6f}")
    link.set_origin(args.origin[0], args.origin[1])
    time.sleep(2.0)
    if not link.set_mode_hold():
        print("  UYARI: Hold moduna geçilemedi")
    time.sleep(1.0)

    # MAV_CMD_NAV_TAKEOFF'un param7'si (irtifa) hangi referansa göre
    # yorumlandığı net değil -- 2.0 istendiğinde araç 1.75 m'de asılı kalıyordu.
    # PX4'ün AUTO kalkışında güvenilir olan MIS_TAKEOFF_ALT parametresi.
    print(f"\nkalkış irtifası ayarlanıyor: {args.altitude:.2f} m")
    if not link.set_param("MIS_TAKEOFF_ALT", args.altitude):
        print("  UYARI: MIS_TAKEOFF_ALT yazılamadı, komut param7'sine güvenilecek")

    print("arm ediliyor...")
    if not link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0):
        link.stop()
        return 1

    print(f"kalkış -> {args.altitude:.1f} m")
    # NaN enlem/boylam = "bulunduğun yerde kalk".
    if not link.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0.0, 0.0, 0.0, float("nan"),
                    float("nan"), float("nan"), args.altitude):
        link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
        link.stop()
        return 1

    # Hedefe "ulaşıldı" demek yerine irtifanın OTURMASINI bekliyoruz.
    # PX4'ün AUTO kalkışı istenen irtifanın sabit ~0.25 m altında dengeleniyor
    # (2.0 -> 1.75, 3.0 -> 2.75 ölçüldü); eşik kontrolü bunu hatalı sanıp
    # yanıltıcı uyarı basıyordu. Gerçek irtifa aşağıda raporlanıyor.
    deadline = time.time() + 60
    settled_at = None
    history: list[float] = []
    while time.time() < deadline:
        z, _ = link.state()
        if z is not None:
            progress(f"irtifa {z:5.2f} m")
            history.append(z)
            recent = history[-15:]
            if len(recent) == 15 and max(recent) - min(recent) < 0.05 and z > 0.5:
                settled_at = z
                break
        time.sleep(0.2)
    print()
    if settled_at is None:
        print("UYARI: irtifa oturmadı, yine de devam ediliyor.")
    else:
        err = settled_at - args.altitude
        print(f"irtifa oturdu: {settled_at:.2f} m "
              f"(istenen {args.altitude:.2f} m, fark {err:+.2f} m)")

    if args.yaw is not None:
        # MAV_CMD_DO_REPOSITION: enlem/boylam/irtifa NaN = "olduğun yerde kal",
        # sadece yaw değişsin. PX4 bunu Hold/Loiter modunda kabul ediyor;
        # kalkış sonrası araç zaten orada.
        print(f"yaw -> {args.yaw:.0f}°")
        if not link.cmd(mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                        -1.0, 0.0, 0.0, math.radians(args.yaw),
                        float("nan"), float("nan"), float("nan")):
            print("  UYARI: yaw komutu kabul edilmedi, yön değişmeden devam")
        else:
            time.sleep(6.0)     # dönüşün oturması

    print(f"{args.hover:.0f} s havada bekleniyor (kare yakalama zamanı)")
    end = time.time() + args.hover
    while time.time() < end:
        z, _ = link.state()
        if z is not None:
            progress(f"irtifa {z:5.2f} m  kalan {end-time.time():4.1f} s")
        time.sleep(0.3)
    print()

    print("iniş...")
    link.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0, float("nan"),
             float("nan"), float("nan"), 0.0)

    deadline = time.time() + 60
    while time.time() < deadline:
        z, armed = link.state()
        if z is not None:
            progress(f"irtifa {z:5.2f} m")
        if armed is False:
            print("\niniş tamam, disarm edildi.")
            break
        time.sleep(0.3)
    else:
        print("\nUYARI: disarm doğrulanamadı.")

    link.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
