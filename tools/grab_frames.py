#!/usr/bin/env python3
"""Gazebo kamera konularından kare yakalar (ROS 2 üzerinden).

SİSTEM PYTHON'U İLE ÇALIŞIR, proje venv'i ile değil -- rclpy ROS kurulumuna
bağlı. Önce ROS ortamını source edin:

    source /opt/ros/jazzy/setup.bash
    python3 tools/grab_frames.py --topics /warehouse_scout/camera_front/image \
        --out out/frames --count 20

gz'nin kamera <save> özelliği bu kurulumda kare yazmıyor (hata da vermiyor),
o yüzden kareler konudan alınıyor. ros_gz köprüsü zaten otonomi ve sentetik
veri aşamasında kullanılacak, dolayısıyla bu geçici bir çözüm değil.

Kareler ham PNG olarak yazılır; çözme işini .venv/bin/python ile
tools/decode_frames.py yapar (pyzbar orada kurulu).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_image import to_array          # noqa: E402


class Grabber(Node):
    def __init__(self, topics: list[str], out: Path, count: int):
        super().__init__("warehouse_frame_grabber")
        self.out, self.count = out, count
        self.saved = {t: 0 for t in topics}
        # Kamera yayınları best-effort; varsayılan RELIABLE ile abonelik eşleşmez
        # ve sessizce hiç kare gelmez.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for t in topics:
            (out / t.strip("/").replace("/", "_")).mkdir(parents=True, exist_ok=True)
            self.create_subscription(Image, t, self._make_cb(t), qos)

    def _make_cb(self, topic: str):
        d = self.out / topic.strip("/").replace("/", "_")

        def cb(msg: Image) -> None:
            if self.saved[topic] >= self.count:
                return
            i = self.saved[topic]
            PILImage.fromarray(to_array(msg)).save(d / f"{i:04d}.png")
            self.saved[topic] = i + 1
            if i == 0:
                self.get_logger().info(f"{topic}: {msg.width}x{msg.height} {msg.encoding}")
        return cb

    @property
    def done(self) -> bool:
        return all(v >= self.count for v in self.saved.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topics", nargs="+", required=True, help="gz kamera konuları")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--count", type=int, default=10, help="konu başına kare")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--no-bridge", action="store_true",
                    help="köprüyü başlatma (dışarıda çalışıyorsa)")
    args = ap.parse_args()

    bridge = None
    if not args.no_bridge:
        bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_image", "image_bridge", *args.topics],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4.0)     # köprünün konuları keşfetmesi için

    rclpy.init()
    node = Grabber(args.topics, args.out, args.count)
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()

    ok = True
    for t, n in node.saved.items():
        print(f"{t}: {n}/{args.count} kare")
        ok &= n > 0
    if not ok:
        print("\nHiç kare gelmedi. Simülasyon çalışıyor mu, konu adları doğru mu?",
              file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
