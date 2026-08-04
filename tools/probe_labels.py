#!/usr/bin/env python3
"""Etiketlerin gerçek Gazebo render'ında okunup okunmadığını ölçer.

Uçuştan bağımsız çalışır: ground_truth.json'dan hedef etiketler seçilir, her
birinin tam karşısına -- normali boyunca istenen mesafeye -- ön kamerayla aynı
özelliklerde bir prob kamerası konur, world birkaç saniye headless koşturulur
ve kaydedilen kareler pyzbar ile çözülür.

Böylece "kod okunuyor mu" sorusu uçuş kontrolünden, otonomiden ve kamera
montaj açısından ayrışıyor: burada bir başarısızlık dokunun, aydınlatmanın
veya ölçünün sorunudur.

    .venv/bin/python tools/probe_labels.py
    .venv/bin/python tools/probe_labels.py --distance 2.0 --type box_qr
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))

# Standalone koşuda PX4'ün server.config'i devrede olmadığı için render ve
# fizik sistem eklentilerini world'e elle koymak gerekiyor. Sensors olmadan
# kamera hiç render etmez ve sessizce boş çıktı alırsınız.
SYSTEM_PLUGINS = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
"""


def probe_model(idx: int, cam: dict, target: dict, distance: float) -> tuple[str, str]:
    """Etiketin normali boyunca `distance` kadar geride, ona bakan kamera.
    (sdf_parçası, konu_adı) döndürür."""
    lx, ly, lz = target["label_pose_xyzrpy"][:3]
    nx, ny, nz = target["normal"]

    px, py, pz = lx + nx * distance, ly + ny * distance, lz + nz * distance

    # Kamera optik ekseni yerelde +X; onu -normal yönüne çevir.
    if abs(nz) > 0.5:                       # zemine bakan (markör)
        roll, pitch, yaw = 0.0, math.pi / 2, 0.0
    else:
        roll, pitch = 0.0, 0.0
        yaw = math.atan2(-ny, -nx)

    topic = f"/probe_{idx:02d}/image"

    sdf = f"""
    <model name="probe_{idx:02d}">
      <static>true</static>
      <pose>{px:.6g} {py:.6g} {pz:.6g} {roll:.6g} {pitch:.6g} {yaw:.6g}</pose>
      <link name="link">
        <sensor name="probe_cam" type="camera">
          <topic>{topic}</topic>
          <camera>
            <horizontal_fov>{cam['hfov']:.6g}</horizontal_fov>
            <image>
              <width>{cam['width']}</width>
              <height>{cam['height']}</height>
              <format>R8G8B8</format>
            </image>
            <clip><near>0.05</near><far>40</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>2</update_rate>
        </sensor>
      </link>
    </model>
"""
    return sdf, topic


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--world", type=Path,
                    default=PROJECT_ROOT / "gz" / "worlds" / "warehouse.sdf")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--distance", type=float, default=1.5,
                    help="etiketten kamera mesafesi (m). koridor merkezi = 1.5")
    ap.add_argument("--type", action="append", dest="types",
                    help="sadece bu tip(ler): box_qr / bay_placard / floor_marker")
    ap.add_argument("--per-type", type=int, default=3, help="tip başına hedef sayısı")
    ap.add_argument("--frames", type=int, default=3, help="hedef başına kare sayısı")
    ap.add_argument("--seconds", type=float, default=30.0, help="kare yakalama zaman aşımı")
    ap.add_argument("--keep", action="store_true", help="kareleri silme")
    args = ap.parse_args()

    import yaml
    from PIL import Image
    from pyzbar import pyzbar

    cfg = yaml.safe_load(args.config.read_text())
    cam = next(c for c in cfg["cameras"] if c["name"] == "front")
    truth = json.loads(args.ground_truth.read_text())["codes"]

    wanted = args.types or ["box_qr", "bay_placard", "floor_marker"]
    targets = []
    for t in wanted:
        of_type = [c for c in truth if c["type"] == t]
        # ortadan seç: kenardaki gözler duvara/koridor sonuna denk gelebiliyor
        mid = len(of_type) // 2
        targets += of_type[mid:mid + args.per_type]
    if not targets:
        print("Hedef bulunamadı.")
        return 1

    work = Path(os.environ.get("TMPDIR", "/tmp")) / "wh_probe"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)

    models, topics = [], []
    for i, tgt in enumerate(targets):
        sdf, topic = probe_model(i, cam, tgt, args.distance)
        models.append(sdf)
        topics.append(topic)

    world_src = args.world.read_text()
    probe_world = world_src.replace("  </world>",
                                    SYSTEM_PLUGINS + "".join(models) + "  </world>")
    world_path = work / "probe_world.sdf"
    world_path.write_text(probe_world)

    env = os.environ.copy()
    env["PATH"] = f"{PROJECT_ROOT}/scripts/bin:" + env.get("PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = ":".join([
        str(PROJECT_ROOT / "gz" / "models"),
        str(PX4_DIR / "Tools" / "simulation" / "gz" / "models"),
    ])
    # PX4'ün server.config'i devreye girmesin; eklentiler world'de tanımlı.
    env.pop("GZ_SIM_SERVER_CONFIG_PATH", None)

    print(f"{len(targets)} hedef, {args.distance:.2f} m mesafeden")
    proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1", str(world_path)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    frames_root = work / "frames"
    try:
        time.sleep(6.0)     # world'ün yüklenip kameraların yayına başlaması
        # Kareler ROS köprüsü üzerinden alınıyor: gz'nin kamera <save>
        # özelliği bu kurulumda sessizce hiçbir şey yazmıyor.
        # Proje yolu boşluk içerebiliyor ("warehouse test"), o yüzden kabuğa
        # geçen her yol shlex ile tırnaklanmalı.
        cmd = " ".join([
            "source /opt/ros/jazzy/setup.bash &&",
            "exec python3", shlex.quote(str(PROJECT_ROOT / "tools" / "grab_frames.py")),
            "--topics", *topics,
            "--out", shlex.quote(str(frames_root)),
            "--count", str(args.frames),
            "--timeout", f"{args.seconds + 30:.0f}",
        ])
        grab = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
        if grab.returncode != 0:
            print(grab.stdout)
            print(grab.stderr, file=sys.stderr)
    finally:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()

    errs = [l for l in (out or "").splitlines()
            if "Error" in l or "Unable" in l or "out of bounds" in l]
    if errs:
        print("\ngz hata satırları:")
        for l in dict.fromkeys(errs).keys():
            print("  " + l[:160])

    print(f"\n{'hedef':<14}{'beklenen':<26}{'kare':>6}{'çözüldü':>9}  sonuç")
    print("-" * 70)
    ok_count = 0
    for tgt, topic in zip(targets, topics):
        d = frames_root / topic.strip("/").replace("/", "_")
        frames = sorted(d.glob("*.png")) if d.exists() else []
        hits = 0
        for f in frames:
            try:
                got = {r.data.decode("utf-8", "replace") for r in pyzbar.decode(Image.open(f))}
            except Exception:
                continue
            if tgt["payload"] in got:
                hits += 1
        verdict = "OK" if hits else ("KARE YOK" if not frames else "OKUNAMADI")
        if hits:
            ok_count += 1
        print(f"{tgt['type']:<14}{tgt['payload']:<26}{len(frames):>6}{hits:>9}  {verdict}")

    print(f"\n{ok_count}/{len(targets)} hedef okundu")
    if args.keep:
        print(f"kareler: {work}")
    elif work.exists():
        shutil.rmtree(work)
    return 0 if ok_count == len(targets) else 1


if __name__ == "__main__":
    raise SystemExit(main())
