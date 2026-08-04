#!/usr/bin/env python3
"""GPS'siz (optik akış + mesafe ölçer) konum kestiriminin kaymasını ölçer.

Yol haritasının 2. aşamasının kanıt aracı: araç kalkar, verilen süre boyunca
havada bekler, iner. Bu sırada iki şey aynı anda kaydedilir:

  * PX4'ün kendi konum kestirimi (MAVLink LOCAL_POSITION_NED)
  * Gazebo'nun gerçek pozu (ground truth)

    source /opt/ros/jazzy/setup.bash    # gerekmez, ama zararı da yok
    .venv/bin/python scripts/measure_drift.py --hover 60

GROUND TRUTH ASLA ALGORİTMAYA GİRDİ DEĞİLDİR. Burada yalnızca "kestirim ne
kadar yanlış" sorusunu cevaplamak için okunuyor; PX4 onu görmüyor. Bu proje
kuralı, kendimizi kandırmamak için var (bkz. README).

Ölçülen üç şey:
  fiziksel savrulma  aracın gerçekten kalkış noktasından ne kadar uzaklaştığı.
                     Kestirim kayarsa konum denetleyicisi hayali bir hatayı
                     kovalar ve araç fiziksel olarak sürüklenir.
  kestirim hatası    aynı anın gerçek pozu ile PX4'ün sandığı poz arasındaki
                     fark.
  sensör akışı       mesafe ölçer ve akış verisi PX4'e GERÇEKTEN ulaşıyor mu.
                     Ulaşmıyorsa kayma ölçümü anlamsızdır -- araç barometre +
                     ataletle uçuyordur ve bunu fark etmeden "iyi sonuç"
                     sanabilirsiniz.

Gazebo'nun ENU dünyası ile PX4'ün NED yerel çerçevesi arasındaki eşleme
varsayılmıyor, ÖLÇÜLÜYOR: iki aday eşlemenin ikisi de hesaplanıp hangisinin
tutarlı olduğu raporlanıyor.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_flight import ORIGIN_LATLON, Link, progress, wait_ready      # noqa: E402

# ENU (Gazebo) -> NED (PX4) aday eşlemeleri. Hangisinin doğru olduğu
# ölçümden çıkarılıyor; varsaymak sessiz bir işaret hatasına yol açardı.
#
# ÖLÇÜLDÜ (2026-08-04): doğru olan "Y=Kuzey". PX4'ün kuzeyi Gazebo'nun +Y'si,
# doğusu +X'i. Yani burnu dünya +X'ine bakan araç PX4'e göre 90° (doğu)
# yönündedir -- Gazebo yaw'ını doğrudan PX4 heading'iyle karşılaştırmak
# sahte bir "90° yön hatası" üretir.
FRAMES = {
    "X=Kuzey": lambda x, y, z: (x, -y, -z),
    "Y=Kuzey": lambda x, y, z: (y, x, -z),
}


def gz_yaw_to_heading(yaw: float) -> float:
    """Gazebo yaw'ı (dünya +X'ten saat yönünün tersi) -> PX4 heading'i (NED,
    kuzeyden saat yönü). Kuzey = +Y olduğu için heading = 90° - yaw."""
    return math.pi / 2 - yaw


class TruthFeed:
    """Gazebo'nun gerçek poz akışı. `gz topic -e --json-output` alt süreci.

    gz-transport'un python bağlayıcısı bu kurulumda yok; CLI'yi boru ile
    okumak tek bağımlılıksız yol. Mesajlar çok satırlı JSON olarak gelebildiği
    için tamponu raw_decode ile parçalıyoruz.
    """

    def __init__(self, world: str, model: str):
        self.model = model
        self.pose: tuple[float, float, float] | None = None
        self.yaw: float | None = None
        self.count = 0
        self._stop = threading.Event()
        env_path = f"{PROJECT_ROOT / 'scripts' / 'bin'}:" + __import__("os").environ["PATH"]
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", f"/world/{world}/dynamic_pose/info",
             "--json-output"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            env={**__import__("os").environ, "PATH": env_path})
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self) -> None:
        dec, buf = json.JSONDecoder(), ""
        while not self._stop.is_set():
            chunk = self._proc.stdout.readline()
            if not chunk:
                break
            buf += chunk
            while buf.strip():
                try:
                    obj, end = dec.raw_decode(buf.lstrip())
                except ValueError:
                    break                       # mesaj henüz tamamlanmadı
                buf = buf.lstrip()[end:]
                self._take(obj)

    def _take(self, obj: dict) -> None:
        for p in obj.get("pose", []):
            # SADECE model girdisi: aynı mesajdaki link girdileri (base_link,
            # rotor_0, ...) dünya değil MODELE GÖRE poz taşıyor, onları
            # ground truth sanmak sabit bir ofset hatası verirdi.
            if p.get("name") == self.model:
                pos = p.get("position", {})
                self.pose = (float(pos.get("x", 0.0)), float(pos.get("y", 0.0)),
                             float(pos.get("z", 0.0)))
                q = p.get("orientation", {})
                qw, qx = float(q.get("w", 0.0)), float(q.get("x", 0.0))
                qy, qz = float(q.get("y", 0.0)), float(q.get("z", 0.0))
                self.yaw = math.atan2(2 * (qw * qz + qx * qy),
                                      1 - 2 * (qy * qy + qz * qz))
                self.count += 1
                return

    def stop(self) -> None:
        self._stop.set()
        self._proc.terminate()
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.kill()


def request_streams(link: Link) -> None:
    """Sensör teşhis mesajlarını iste. Varsayılan akışta gelmiyorlar."""
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0),
                       (mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 5.0)):
        link.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                 float(msg_id), 1e6 / hz)


def rms(vals: list[float]) -> float:
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--altitude", type=float, default=2.0)
    ap.add_argument("--hover", type=float, default=60.0, help="ölçüm süresi (s)")
    ap.add_argument("--yaw", type=float, help="kalkıştan sonra bu yöne dön (derece)")
    ap.add_argument("--square", type=float, default=0.0,
                    help="ölçüm sırasında bu kenar uzunluğunda (m) kare çiz. "
                         "0 = yerinde askıda kal. Askıda kalmak kaymayı "
                         "GÖSTERMEZ: optik akış hata biriktirmek için hareket "
                         "ister. Koridor 3 m geniş, kenarı 1.5 m'nin üstüne "
                         "çıkarmayın.")
    ap.add_argument("--leg", type=float, default=6.0,
                    help="karenin her köşesinde bekleme süresi (s)")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--model", default="warehouse_scout_0")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "out" / "nav")
    args = ap.parse_args()

    truth = TruthFeed(args.world, args.model)
    link = Link(args.url)
    link.start()
    wait_ready(link)
    request_streams(link)

    time.sleep(1.0)
    ground = truth.pose                 # kalkıştan ÖNCEKİ referans
    if truth.pose is None:
        print("HATA: Gazebo'dan gerçek poz gelmedi. World adı doğru mu "
              f"({args.world}), model adı doğru mu ({args.model})?", file=sys.stderr)
        truth.stop()
        link.stop()
        return 1
    print(f"gerçek poz akışı çalışıyor ({truth.count} mesaj)")

    # GPS kapalı -> origin ve Hold modu olmadan arm reddedilir (test_flight.py
    # ile aynı gerekçe, orada belgeli).
    link.set_origin(*ORIGIN_LATLON)
    time.sleep(2.0)
    link.set_mode_hold()
    time.sleep(1.0)

    print(f"\nkalkış irtifası: {args.altitude:.2f} m")
    link.set_param("MIS_TAKEOFF_ALT", args.altitude)
    if not link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0):
        truth.stop(); link.stop()
        return 1
    if not link.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0,
                    float("nan"), float("nan"), float("nan"), args.altitude):
        link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
        truth.stop(); link.stop()
        return 1
    time.sleep(12.0)                    # kalkışın oturması

    if args.yaw is not None:
        print(f"yaw -> {args.yaw:.0f}°")
        link.cmd(mavutil.mavlink.MAV_CMD_DO_REPOSITION, -1.0, 0.0, 0.0,
                 math.radians(args.yaw), float("nan"), float("nan"), float("nan"))
        time.sleep(6.0)

    origin_truth = truth.pose
    corners: list[tuple[float, float]] = []
    if args.square > 0:
        h = args.square / 2.0
        corners = [(h, h), (h, -h), (-h, -h), (-h, h)]
    next_corner, corner_due = 0, time.time()
    rows = []
    ranges: list[float] = []
    range_orient: set[int] = set()
    yaw_err: list[float] = []
    print(f"\n{args.hover:.0f} s ölçüm (ground truth yalnızca kayıt için)")
    end = time.time() + args.hover
    t0 = time.time()
    while time.time() < end:
        if corners and time.time() >= corner_due:
            # Köşeler PX4'ün KENDİ yerel çerçevesinde veriliyor (lat/lon'a
            # çevrilerek). Yön kestirimi dünyaya göre dönükse kare de dönük
            # çizilir -- bu kasıtlı: amaç aracı belirli bir dünya noktasına
            # götürmek değil, akışa hata biriktirtmek.
            dn, de = corners[next_corner % len(corners)]
            lat = ORIGIN_LATLON[0] + dn / 111320.0
            lon = ORIGIN_LATLON[1] + de / (111320.0 * math.cos(
                math.radians(ORIGIN_LATLON[0])))
            link.m.mav.command_long_send(
                link.m.target_system, link.m.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0,
                -1.0, 0.0, 0.0, float("nan"), lat, lon, args.altitude)
            next_corner += 1
            corner_due = time.time() + args.leg

        msg = link.m.recv_match(
            type=["LOCAL_POSITION_NED", "DISTANCE_SENSOR", "ATTITUDE"],
            blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "DISTANCE_SENSOR":
            ranges.append(msg.current_distance / 100.0)
            range_orient.add(msg.orientation)
            continue
        if msg.get_type() == "ATTITUDE":
            if truth.yaw is not None:
                # Kestirilen yön ile gerçek yön farkı. Akış temelli
                # navigasyonda yön hatası, hız vektörünü döndürüp konumu
                # yanlış yöne biriktirdiği için kaymanın en büyük kaynağı.
                # Gerçek yaw önce PX4'ün çerçevesine çevriliyor; ham
                # karşılaştırma 90°'lik sahte bir hata verir.
                ref = gz_yaw_to_heading(truth.yaw)
                yaw_err.append(math.degrees(
                    (msg.yaw - ref + math.pi) % (2 * math.pi) - math.pi))
            continue
        tp = truth.pose
        if tp is None:
            continue
        t = time.time() - t0
        rows.append((t, *tp, msg.x, msg.y, msg.z))
        dx, dy = tp[0] - origin_truth[0], tp[1] - origin_truth[1]
        progress(f"t={t:5.1f}s  savrulma {math.hypot(dx, dy):5.2f} m  "
                 f"irtifa(gerçek) {tp[2]:5.2f} m  kestirim z {-msg.z:5.2f} m")
    print()

    print("iniş...")
    link.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0, float("nan"),
             float("nan"), float("nan"), 0.0)
    deadline = time.time() + 60
    while time.time() < deadline:
        _, armed = link.state()
        if armed is False:
            break
        time.sleep(0.3)
    truth.stop()
    link.stop()

    if not rows:
        print("HATA: hiç ölçüm alınamadı.", file=sys.stderr)
        return 1

    # ---------------------------------------------------------------- rapor
    args.out.mkdir(parents=True, exist_ok=True)
    csv = args.out / f"drift_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with csv.open("w") as f:
        f.write("t,truth_x,truth_y,truth_z,ekf_n,ekf_e,ekf_d\n")
        for r in rows:
            f.write(",".join(f"{v:.4f}" for v in r) + "\n")

    ox, oy, oz = origin_truth
    wander = [math.hypot(r[1] - ox, r[2] - oy) for r in rows]
    n0, e0, d0 = rows[0][4], rows[0][5], rows[0][6]

    print()
    print(f"süre                : {rows[-1][0]:.0f} s, {len(rows)} örnek")
    print(f"fiziksel savrulma   : son {wander[-1]:.2f} m, en fazla {max(wander):.2f} m")
    print(f"  (kalkış noktasına göre yatay; kestirim kayarsa denetleyici "
          f"aracı fiziksel olarak buraya sürükler)")

    print("\nkestirim hatası (gerçek - PX4), iki aday çerçeve:")
    best = None
    for name, fn in FRAMES.items():
        errs = []
        for t, tx, ty, tz, n, e, d in rows:
            rn, re_, rd = fn(tx - ox, ty - oy, tz - oz)
            errs.append(math.hypot(rn - (n - n0), re_ - (e - e0)))
        r = rms(errs)
        print(f"  {name:10s} yatay RMS {r:6.3f} m   son {errs[-1]:6.3f} m")
        if best is None or r < best[1]:
            best = (name, r, errs)
    print(f"  -> tutarlı olan: {best[0]} (diğeri işaret/eksen hatası verir)")

    # Dikey referans KALKIŞ ÖNCESİ yerden alınır: ölçüm penceresinin ilk
    # anını referans almak, tırmanış henüz oturmamışsa sahte bir sabit hata
    # üretiyordu.
    gz_ = ground[2]
    vert = [abs((r[3] - gz_) - (-r[6])) for r in rows]
    print(f"\ndikey hata          : RMS {rms(vert):.3f} m, son {vert[-1]:.3f} m")
    print(f"  (yerdeki referansa göre: gerçek yükseklik {rows[-1][3] - gz_:.2f} m, "
          f"PX4 {-rows[-1][6]:.2f} m)")

    if yaw_err:
        print(f"\nyön (yaw) hatası    : ortalama {sum(yaw_err)/len(yaw_err):+.1f}°, "
              f"son {yaw_err[-1]:+.1f}°")
        if abs(sum(yaw_err) / len(yaw_err)) > 5:
            print("  UYARI: yön hatası büyük. Kalan kısım manyetik sapma "
                  "(declination) kaynaklı; rotayı dünya koordinatına oturtmak "
                  "için 3. aşamadaki AprilTag düzeltmesi gerekiyor.")

    print(f"\nmesafe ölçer        : {len(ranges)} ölçüm", end="")
    if ranges:
        ranges_sorted = sorted(ranges)
        print(f", {ranges_sorted[0]:.2f}-{ranges_sorted[-1]:.2f} m, "
              f"medyan {ranges_sorted[len(ranges)//2]:.2f} m")
        names = {25: "DOWNWARD_FACING", 0: "FORWARD", 12: "CUSTOM"}
        for o in sorted(range_orient):
            tag = names.get(o, f"kod {o}")
            warn = "" if o == 25 else "   <-- EKF2 BUNU KULLANMAZ!"
            print(f"  yönelim: {tag}{warn}")
    else:
        print("\n  UYARI: hiç mesafe ölçümü gelmedi -- EKF2 mesafe ölçeri "
              "kullanmıyor demektir.")

    print(f"\nCSV: {csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
