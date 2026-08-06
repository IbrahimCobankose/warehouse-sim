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


def hover_rotate(ob, link, pos_ned, d, from_h, to_h, rate_deg=30.0):
    """Yerinde (hover) dönüş: konumu `pos_ned`de sabit tutup burnu `from_h`ten
    `to_h`e en kısa yoldan, `rate_deg`/s ile döndür. Büyük yaw dönüşlerini
    translasyondan AYIRIR: optik akış dönerken bozuluyor (2. aşama), o yüzden
    dönüş ya tag üstünde (AprilTag düzeltmesi var) ya açık alanda (raf uzakta)
    yapılmalı -- ikisinde de konum bu setpoint'le tutuluyor."""
    delta = (to_h - from_h + math.pi) % (2.0 * math.pi) - math.pi
    dur = abs(delta) / math.radians(rate_deg)
    print(f"\n  yerinde dönüş {math.degrees(from_h):+.0f}° -> "
          f"{math.degrees(to_h):+.0f}° ({dur:.1f} s)")
    t0 = time.time()
    while True:
        f = min(1.0, (time.time() - t0) / dur) if dur > 1e-3 else 1.0
        ob.set(pos_ned[0], pos_ned[1], d, from_h + delta * f)
        if f >= 1.0:
            break
        link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
    # dönüş bitince kısa oturma: yeni yönde dengelensin
    settle = time.time() + 1.5
    while time.time() < settle:
        ob.set(pos_ned[0], pos_ned[1], d, to_h)
        link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)


def hover_climb(ob, link, pos_ned, heading, from_d, to_d, rate=0.4):
    """Yerinde irtifa değişimi: konumu ve burnu sabit tutup irtifayı `from_d`ten
    `to_d`e `rate` m/s ile değiştir (d = NED aşağı = -irtifa). Seviyeler arası
    geçiş için: kutuların önünden geçerken DEĞİL, geçişler ARASINDA irtifa
    değiştir -- geçerken değiştirmek okumayı bozar. hover_rotate'in irtifa
    kardeşi; her raf yüzü 3 seviyede tarandığından (12 geçiş) gerekli."""
    dur = abs(to_d - from_d) / rate if rate > 1e-6 else 0.0
    print(f"\n  yerinde irtifa {-from_d:.2f} -> {-to_d:.2f} m ({dur:.1f} s)")
    t0 = time.time()
    while True:
        f = min(1.0, (time.time() - t0) / dur) if dur > 1e-3 else 1.0
        ob.set(pos_ned[0], pos_ned[1], from_d + (to_d - from_d) * f, heading)
        if f >= 1.0:
            break
        link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
    settle = time.time() + 1.5
    while time.time() < settle:
        ob.set(pos_ned[0], pos_ned[1], to_d, heading)
        link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)


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
    route_yaw = float(r["yaw"])
    alt = float(r["altitude"])
    spawn = np.array(cfg["spawn"]["pose"][:3], dtype=np.float64)
    tags, _ = load_tag_map(args.ground_truth, cfg)

    # Dünya -> yerel NED. Kuzey = dünya +Y, Doğu = dünya +X.
    def to_ned(world_xy):
        return np.array([world_xy[1] - spawn[1], world_xy[0] - spawn[0]])

    # Waypoint biçimleri (bir rotada karışabilir):
    #   0                      -> çıplak tag; yaw = rotanın genel `yaw`ı
    #   {tag: 4, yaw: -90}     -> tag + kendi yaw'ı
    #   {xy: [8.5, -3.4]}      -> tag OLMAYAN dünya noktası (rafın ucunu
    #                             dolaşan köşe noktaları için, 5. aşama dönüşü)
    #   {..., spin: 0}         -> VARIŞTA yerinde dön: burnu bu noktanın
    #                             yaw'ından `spin` dereceye HOVER'da çevir.
    #                             Büyük dönüşleri translasyondan ayırır.
    #   {..., log:.., hold:..} -> tag'e bağlı `actions` yerine satır-içi eylem
    # yaw verilmezse bir önceki noktanın (varsa spin sonrası) yaw'ına düşer;
    # böylece bacaklar SABİT yönle uçulur ve dönüş yalnızca spin noktalarında
    # olur -- eski çıplak-int rotalar (koridor1_A_seviye2) değişmeden çalışır.
    # `alt` (waypoint başına KALKIŞ irtifası, 6. aşama): o noktadan sonraki
    # bacak bu irtifada uçulur; irtifa değişirse varışta YERİNDE değişir
    # (hover_climb). Verilmezse bir öncekine düşer -> tek-irtifa rotalar
    # (koridor1_A_seviye2) değişmeden çalışır. Her raf yüzü 3 seviyede
    # taranacağı için (12 geçiş) aynı koridorda ileri-geri geçişlerde alt
    # değiştirilir.
    wp_world: list = []
    wp_labels: list[str] = []
    wp_yaws: list[float] = []
    wp_spins: list = []          # varışta yerinde dönülecek hedef derece / None
    wp_alts: list[float] = []    # o noktadan sonraki bacağın irtifası
    wp_action: list = []         # {log?, hold?} / None
    prev_yaw, prev_alt = route_yaw, alt
    for entry in r["waypoints"]:
        if isinstance(entry, dict) and "xy" in entry:
            wx, wy = float(entry["xy"][0]), float(entry["xy"][1])
            label, act = f"xy({wx:+5.1f},{wy:+5.1f})", {}
        else:
            t = int(entry["tag"]) if isinstance(entry, dict) else int(entry)
            if t not in tags:
                print(f"Haritada olmayan tag: {t}", file=sys.stderr)
                return 1
            wx, wy = float(tags[t][0]), float(tags[t][1])
            label, act = f"tag {t:>2}       ", dict(actions.get(t) or {})
        if isinstance(entry, dict):
            y = float(entry.get("yaw", prev_yaw))
            spin = float(entry["spin"]) if "spin" in entry else None
            a_val = float(entry.get("alt", prev_alt))
            for k in ("log", "hold"):
                if k in entry:
                    act[k] = entry[k]
        else:
            y, spin, a_val = route_yaw, None, prev_alt
        wp_world.append((wx, wy))
        wp_labels.append(label)
        wp_yaws.append(y)
        wp_spins.append(spin)
        wp_alts.append(a_val)
        wp_action.append(act or None)
        prev_yaw = spin if spin is not None else y
        prev_alt = a_val

    wps = [to_ned(w) for w in wp_world]
    wp_headings = [gz_yaw_to_heading(y) for y in wp_yaws]
    # Bacaktan ÇIKIŞ yönü: nokta spin ediyorsa spin hedefi, yoksa giriş yönü.
    depart_headings = [gz_yaw_to_heading(s) if s is not None else h
                       for s, h in zip(wp_spins, wp_headings)]

    multi_alt = len(set(wp_alts)) > 1
    varying = len(set(wp_yaws)) > 1 or any(s is not None for s in wp_spins) or multi_alt
    print(f"rota      : {args.route}  ({len(wps)} nokta)")
    if varying:
        print(f"yaw       : waypoint başına (aşağıda), genel {route_yaw:.0f}°")
    else:
        print(f"yaw       : {route_yaw:.0f}° (dünya) -> "
              f"{math.degrees(wp_headings[0]):.0f}° (NED)")
    slow_txt = (f"   yavaşlama: {slow_radius:.1f} m kala x{slow_floor:.2f}"
                if slow_radius > 0 else "")
    alt_txt = (f"{min(wp_alts):.2f}-{max(wp_alts):.2f} m (waypoint başına)"
               if multi_alt else f"{alt:.2f} m")
    print(f"irtifa    : {alt_txt}   hız: {speed:.2f} m/s{slow_txt}")
    for lab, w, y, s, a in zip(wp_labels, wps, wp_yaws, wp_spins, wp_alts):
        extra = ""
        if varying:
            extra = f"  yaw {y:+.0f}°"
            if s is not None:
                extra += f" -> yerinde {s:+.0f}°"
            if multi_alt:
                extra += f"  alt {a:.2f}"
        print(f"  {lab}  NED ({w[0]:+6.2f}, {w[1]:+6.2f}){extra}")
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
    cur_alt = wp_alts[0]
    ob.set(wps[0][0], wps[0][1], -cur_alt, heading)
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
    print(f"tırmanış -> {cur_alt:.2f} m")
    climb_deadline = time.time() + 40
    while time.time() < climb_deadline:
        msg = link.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue
        progress(f"irtifa {-msg.z:5.2f} m")
        if -msg.z >= cur_alt - 0.25:
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

    def do_action(i: int) -> float:
        a = wp_action[i]
        if not a or i in done_actions:
            return 0.0
        done_actions.add(i)
        if a.get("log"):
            print(f"\n  [{wp_labels[i].strip()}] {a['log']}")
        return float(a.get("hold", 0.0))

    do_action(0)

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
            ob.set(pos[0], pos[1], -cur_alt, heading)
            aborted = True
            break

        remaining = float(np.linalg.norm(b - pos))

        # waypoint başına yaw: bacak boyunca aracın ilerleme oranına göre
        # interpolasyon (dönüş manevraları için). Oran, pozun a->b üstüne
        # izdüşümü -- carrot değil poz, çünkü hedeflenen değil ulaşılan yaw.
        ab = b - a
        L2 = float(ab @ ab)
        f = float((pos - a) @ ab) / L2 if L2 > 1e-9 else 1.0
        heading = lerp_angle(depart_headings[leg], wp_headings[leg + 1], f)

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
        ob.set(carrot[0], carrot[1], -cur_alt, heading)

        progress(f"nokta {leg+1}/{len(wps)-1}  kalan {remaining:5.2f} m  "
                 f"sapma {xt:4.2f} m  hız {v:4.2f}  irtifa {-msg.z:4.2f} m")

        if remaining <= tol:
            hold = do_action(leg + 1)
            leg += 1
            # varışta yerinde dönüş (spin): burnu bu noktada, HOVER'da çevir.
            # Böylece büyük yaw dönüşü translasyondan ayrılır ve ancak güvenli
            # yerde (tag üstünde ya da açık alanda) yapılır.
            if wp_spins[leg] is not None:
                hover_rotate(ob, link, wps[leg], -cur_alt,
                             heading, depart_headings[leg])
                heading = depart_headings[leg]
                carrot = wps[leg].copy()
                t_prev = time.time()
            # varışta yerinde irtifa değişimi: sonraki bacak farklı seviyedeyse
            # burada, kutuların önünden geçmeden önce değiştir (12 geçiş).
            if abs(wp_alts[leg] - cur_alt) > 1e-3:
                hover_climb(ob, link, wps[leg], heading, -cur_alt, -wp_alts[leg])
                cur_alt = wp_alts[leg]
                carrot = wps[leg].copy()
                t_prev = time.time()
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
