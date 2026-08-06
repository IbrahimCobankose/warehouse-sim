#!/usr/bin/env python3
"""`config/route.yaml`'daki rotayı offboard konum kontrolüyle uçurur.

Yol haritasının 4. aşaması. 3. aşama "neredeyim" sorusunu çözdü; burada
"nereye gideceğim" var. AprilTag'ler yalnızca kimlik taşır -- hangi tag'de
ne yapılacağı route.yaml'da (bkz. o dosyanın başındaki gerekçe).

    # 1. terminal
    ./scripts/run_sim.sh
    # 2. terminal -- konum düzeltmesi (3. aşama) ŞART
    source /opt/ros/jazzy/setup.bash
    .venv/bin/python scripts/apriltag_localize.py
    # 3. terminal
    .venv/bin/python scripts/fly_route.py koridor1_A_seviye2

NASIL UÇAR
  Havuç (carrot) yöntemi: hedef nokta doğrudan verilmiyor, ara nokta
  `speed` m/s ile yol boyunca kaydırılıyor. Böylece hız rota dosyasından
  denetleniyor -- PX4'e uzak bir nokta verip MPC_XY_VEL_MAX'e bırakmak
  koridorda çok hızlı ve savrulmalı olurdu.

ÇERÇEVE
  Rota dünya (Gazebo) koordinatlarında yazılı; PX4 yerel NED istiyor.
  Kuzey = dünya +Y, Doğu = dünya +X (ölçüldü, bkz. README). Yerel origin
  kalkış noktası, yani config/warehouse.yaml -> spawn.

GÜVENLİK
  Koridor 3 m geniş, raf yüzleri merkeze ±1.5 m. Betik her adımda koridor
  merkezine dik uzaklığı (cross-track) ölçüyor ve `--abort-offset`i aşarsa
  rotayı kesip olduğu yerde tutuyor. 2. aşamada kestirim patlayıp aracı
  rafa çarptırmıştı; bu o senaryonun frenidir.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_labels as gl                                     # noqa: E402
from apriltag_localize import load_tag_map                  # noqa: E402
from test_flight import ORIGIN_LATLON, Link, progress, wait_ready   # noqa: E402

SETPOINT_HZ = 20.0

# Konum + yaw kullan; hız, ivme ve yaw_rate alanlarını yok say.
TYPE_MASK_POS_YAW = 0b0000_1001_1111_1000


def gz_yaw_to_heading(yaw_deg: float) -> float:
    """Gazebo dünya yaw'ı (derece) -> PX4 NED heading'i (radyan).
    Kuzey = dünya +Y olduğu için heading = 90° - yaw."""
    return math.radians(90.0 - yaw_deg)


def lerp_angle(a: float, b: float, f: float) -> float:
    """İki açı (radyan) arasında en kısa yoldan interpolasyon; f [0,1]'e
    kırpılır. Sarmalamayı doğru çözer (ör. -170° -> +170° kısa yoldan)."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + d * max(0.0, min(1.0, f))


class Offboard:
    """Kesintisiz setpoint akışı. PX4 offboard modunda >2 Hz setpoint
    görmezse moddan düşer, bu yüzden akış ayrı bir iş parçacığında."""

    def __init__(self, link: Link):
        self.link = link
        self.target = None              # (n, e, d, heading)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def set(self, n, e, d, heading):
        self.target = (float(n), float(e), float(d), float(heading))

    def _run(self):
        while not self._stop.is_set():
            t = self.target
            if t is not None:
                self.link.m.mav.set_position_target_local_ned_send(
                    int(time.time() * 1000) & 0xFFFFFFFF,
                    self.link.m.target_system, self.link.m.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    TYPE_MASK_POS_YAW,
                    t[0], t[1], t[2], 0, 0, 0, 0, 0, 0, t[3], 0)
            time.sleep(1.0 / SETPOINT_HZ)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()


def cross_track(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """`p`nin a->b doğrusuna dik uzaklığı (2B)."""
    ab = b - a
    n = np.linalg.norm(ab)
    if n < 1e-6:
        return float(np.linalg.norm(p - a))
    u, d = ab / n, p - a
    # 2B çapraz çarpım elle: numpy 2'de np.cross artık 2B vektör kabul
    # etmiyor (ValueError).
    return float(abs(u[0] * d[1] - u[1] * d[0]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route", help="route.yaml içindeki rota adı")
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--routes", type=Path,
                    default=PROJECT_ROOT / "config" / "route.yaml")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--abort-offset", type=float, default=1.0,
                    help="koridor merkezine dik sapma bu metreyi aşarsa dur "
                         "(raf yüzü 1.5 m'de)")
    ap.add_argument("--dry-run", action="store_true",
                    help="uçurma, sadece rotayı çöz ve yazdır")
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    routes = yaml.safe_load(args.routes.read_text())
    if args.route not in routes["routes"]:
        print(f"Rota yok: {args.route}. Olanlar: {list(routes['routes'])}",
              file=sys.stderr)
        return 1
    r = routes["routes"][args.route]
    speed = float(routes["speed"])
    tol = float(routes["waypoint_tolerance"])
    # Hedefe yaklaşırken oransal yavaşlama (5. aşama ön koşulu). slow_radius=0
    # ise eski davranış (sabit hız). Waypoint'e `slow_radius` m kala hız
    # `speed * slow_floor`a kadar oransal düşürülür.
    slow_radius = float(routes.get("slow_radius", 0.0))
    slow_floor = float(routes.get("slow_floor", 0.2))
    actions = {int(k): v for k, v in (r.get("actions") or {}).items()}

    # Waypoint'ler ya çıplak tag kimliği (int) ya da {tag, yaw} sözlüğü.
    # İkinci biçim waypoint başına yaw taşır (dönüş manevraları, 5. aşama);
    # yaw verilmezse rotanın genel `yaw`ına düşer. Böylece eski, tek-yaw'lı
    # rotalar (koridor1_A_seviye2) değişmeden çalışır.
    route_yaw = float(r["yaw"])
    wp_tags: list[int] = []
    wp_yaws: list[float] = []
    for entry in r["waypoints"]:
        if isinstance(entry, dict):
            wp_tags.append(int(entry["tag"]))
            wp_yaws.append(float(entry.get("yaw", route_yaw)))
        else:
            wp_tags.append(int(entry))
            wp_yaws.append(route_yaw)

    tags, _ = load_tag_map(args.ground_truth, cfg)
    missing = [t for t in wp_tags if t not in tags]
    if missing:
        print(f"Haritada olmayan tag: {missing}", file=sys.stderr)
        return 1

    spawn = np.array(cfg["spawn"]["pose"][:3], dtype=np.float64)
    alt = float(r["altitude"])
    wp_headings = [gz_yaw_to_heading(y) for y in wp_yaws]

    # Dünya -> yerel NED. Kuzey = dünya +Y, Doğu = dünya +X.
    def to_ned(world_xy):
        return np.array([world_xy[1] - spawn[1], world_xy[0] - spawn[0]])

    wps = [to_ned(tags[t][:2]) for t in wp_tags]
    varying_yaw = len(set(wp_yaws)) > 1
    print(f"rota      : {args.route}  ({len(wps)} nokta)")
    if varying_yaw:
        print(f"yaw       : waypoint başına (aşağıda), genel {route_yaw:.0f}°")
    else:
        print(f"yaw       : {route_yaw:.0f}° (dünya) -> "
              f"{math.degrees(wp_headings[0]):.0f}° (NED)")
    slow_txt = (f"   yavaşlama: {slow_radius:.1f} m kala x{slow_floor:.2f}"
                if slow_radius > 0 else "")
    print(f"irtifa    : {alt:.2f} m   hız: {speed:.2f} m/s{slow_txt}")
    for t, w, y in zip(wp_tags, wps, wp_yaws):
        extra = f"  yaw {y:+.0f}°" if varying_yaw else ""
        print(f"  tag {t:>2}  dünya ({tags[t][0]:+6.2f}, {tags[t][1]:+6.2f})"
              f"  -> NED ({w[0]:+6.2f}, {w[1]:+6.2f}){extra}")
    total = sum(float(np.linalg.norm(wps[i + 1] - wps[i])) for i in range(len(wps) - 1))
    print(f"toplam yol: {total:.1f} m, tahmini süre {total/speed:.0f} s")
    if args.dry_run:
        return 0

    link = Link(args.url)
    link.start()
    wait_ready(link)
    link.set_origin(*ORIGIN_LATLON)
    time.sleep(2.0)

    ob = Offboard(link)
    # Offboard'a geçmeden ÖNCE setpoint akmalı: PX4 mod değişimini ancak
    # geçerli bir setpoint görüyorsa kabul ediyor.
    heading = wp_headings[0]
    ob.set(wps[0][0], wps[0][1], -alt, heading)
    ob.start()
    time.sleep(1.5)

    print("\noffboard moduna geçiliyor...")
    if not link.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    6.0, 0.0):
        print("HATA: offboard reddedildi.", file=sys.stderr)
        ob.stop(); link.stop()
        return 1
    if not link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0):
        print("HATA: arm reddedildi.", file=sys.stderr)
        ob.stop(); link.stop()
        return 1

    # ---- tırmanış: ilk noktanın üstüne çık, sonra rotaya gir
    print(f"tırmanış -> {alt:.2f} m")
    climb_deadline = time.time() + 40
    while time.time() < climb_deadline:
        msg = link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue
        progress(f"irtifa {-msg.z:5.2f} m")
        if -msg.z >= alt - 0.25:
            break
    print()

    # ---- rota
    print("rota izleniyor")
    carrot = np.array(wps[0], dtype=np.float64)
    leg = 0
    aborted = False
    max_xt, max_xt_where = 0.0, (0, 0.0)
    t_prev = time.time()
    done_actions: set[int] = set()

    def do_action(tag_id: int) -> float:
        a = actions.get(tag_id)
        if not a or tag_id in done_actions:
            return 0.0
        done_actions.add(tag_id)
        if a.get("log"):
            print(f"\n  [tag {tag_id}] {a['log']}")
        return float(a.get("hold", 0.0))

    do_action(wp_tags[0])

    while leg < len(wps) - 1:
        msg = link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        now = time.time()
        dt, t_prev = now - t_prev, now
        if msg is None:
            continue
        pos = np.array([msg.x, msg.y])

        a, b = wps[leg], wps[leg + 1]
        xt = cross_track(pos, a, b)
        if xt > max_xt:
            # Nerede olduğu önemli: rotanın başındaki bir sapma (tırmanış +
            # yaw oturması) ile koridor ortasındaki bir sapma çok farklı
            # şeyler söyler.
            max_xt, max_xt_where = xt, (leg, float(np.linalg.norm(pos - a)))
        if xt > args.abort_offset:
            print(f"\nDURDURULDU: koridor merkezinden {xt:.2f} m saptı "
                  f"(sınır {args.abort_offset:.2f} m). Olduğu yerde tutuluyor.")
            ob.set(pos[0], pos[1], -alt, heading)
            aborted = True
            break

        remaining = float(np.linalg.norm(b - pos))

        # waypoint başına yaw: bacak boyunca aracın ilerleme oranına göre
        # interpolasyon (dönüş manevraları için). Oran, pozun a->b üstüne
        # izdüşümü -- carrot değil poz, çünkü hedeflenen değil ulaşılan yaw.
        ab = b - a
        L2 = float(ab @ ab)
        f = float((pos - a) @ ab) / L2 if L2 > 1e-9 else 1.0
        heading = lerp_angle(wp_headings[leg], wp_headings[leg + 1], f)

        # hedefe yaklaşırken oransal yavaşlama: son slow_radius m'de hızı
        # slow_floor'a kadar düşür. Overshoot'u ve 4. aşamada bir koşuda
        # görülen 0.85 m sapma aykırısını azaltmak için.
        v = speed
        if slow_radius > 0:
            v = speed * max(slow_floor, min(1.0, remaining / slow_radius))
        step = v * min(dt, 0.5)

        # havucu ilerlet
        seg = b - carrot
        d = float(np.linalg.norm(seg))
        carrot = b.copy() if d <= step else carrot + seg / d * step
        ob.set(carrot[0], carrot[1], -alt, heading)

        progress(f"nokta {leg+1}/{len(wps)-1}  kalan {remaining:5.2f} m  "
                 f"sapma {xt:4.2f} m  hız {v:4.2f}  irtifa {-msg.z:4.2f} m")

        if remaining <= tol:
            hold = do_action(wp_tags[leg + 1])
            leg += 1
            if hold > 0:
                print(f"  {hold:.0f} s bekleniyor")
                end = time.time() + hold
                while time.time() < end:
                    link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                      timeout=1)
                t_prev = time.time()
    print()

    print("iniş...")
    ob.stop()
    link.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0, float("nan"),
             float("nan"), float("nan"), 0.0)
    deadline = time.time() + 60
    while time.time() < deadline:
        _, armed = link.state()
        if armed is False:
            print("iniş tamam, disarm edildi.")
            break
        time.sleep(0.3)
    link.stop()

    print(f"\ntamamlanan nokta  : {leg}/{len(wps)-1}")
    print(f"en fazla sapma    : {max_xt:.2f} m (sınır {args.abort_offset:.2f}, "
          f"raf yüzü 1.5 m)")
    print(f"  sapmanın yeri   : {max_xt_where[0]+1}. bacak, başından "
          f"{max_xt_where[1]:.1f} m sonra")
    return 1 if aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
