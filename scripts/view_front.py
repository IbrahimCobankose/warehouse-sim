#!/usr/bin/env python3
"""Ön kamera CANLI PENCERE + işleme overlay'i.

Uçuş sırasında ön kameranın gördüğünü ve kodun o kare üzerinde yaptığı
İŞLEMEYİ ayrı bir pencerede canlı gösterir:
  * RAF AprilTag tespiti (aruco, apriltag_localize.py'daki lokalizasyon
    dedektörünün aynısı) -> yeşil çerçeve + tag id.
  * KUTU QR tespiti (pyzbar, scan_boxes.py'daki tarayıcının aynısı) -> sarı
    poligon + yük metni.

    source /opt/ros/jazzy/setup.bash
    .venv/bin/python scripts/view_front.py --no-bridge

Pencere için OpenCV'nin GUI'li (opencv-python, headless DEĞİL) kurulu olması
gerekir; venv'de öyle. apriltag_localize ya da scan_boxes zaten ön kamerayı
köprülüyorsa fazladan köprü açma: --no-bridge. PX4 GEREKMEZ. Kapatmak: q / ESC.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_image import to_array, to_gray                       # noqa: E402

FRONT_TOPIC = "/warehouse_scout/camera_front/image"
WIN = "on kamera (canli) -- yesil: raf AprilTag  sari: kutu QR"


def draw_tags(bgr, corners, ids) -> int:
    """Aruco raf tag'lerini çiz (apriltag_localize'ın gördüğü aynı tespit)."""
    if ids is None or len(ids) == 0:
        return 0
    cv2.aruco.drawDetectedMarkers(bgr, corners, ids, (0, 255, 0))
    for c, i in zip(corners, ids.ravel()):
        p = c.reshape(4, 2).mean(axis=0).astype(int)
        cv2.putText(bgr, f"tag {int(i)}", (p[0] - 10, p[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return len(ids)


def draw_qrs(bgr, results) -> int:
    """pyzbar QR sonuçlarını çiz (scan_boxes'ın gördüğü aynı tespit)."""
    n = 0
    for r in results:
        if r.type != "QRCODE":
            continue
        n += 1
        if r.polygon:
            pts = np.array([[p.x, p.y] for p in r.polygon], dtype=np.int32)
        else:
            x, y, w, h = r.rect.left, r.rect.top, r.rect.width, r.rect.height
            pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
                           dtype=np.int32)
        cv2.polylines(bgr, [pts], True, (0, 220, 255), 2)
        payload = r.data.decode("utf-8", "replace")
        top = pts[pts[:, 1].argmin()]
        cv2.putText(bgr, payload, (int(top[0]), int(top[1]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=FRONT_TOPIC)
    ap.add_argument("--no-bridge", action="store_true",
                    help="ros_gz_image köprüsü açma (apriltag/scan_boxes açtıysa)")
    ap.add_argument("--no-tags", action="store_true", help="raf AprilTag overlay kapat")
    ap.add_argument("--no-qr", action="store_true", help="kutu QR overlay kapat")
    ap.add_argument("--scale", type=float, default=0.6,
                    help="pencere ölçeği (1920x1080 ekrana sığsın diye)")
    args = ap.parse_args()

    det = None if args.no_tags else cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters())

    pyzbar = zbar_qr = None
    if not args.no_qr:
        try:
            from pyzbar import pyzbar as _pyzbar
            from pyzbar.pyzbar import ZBarSymbol
            pyzbar, zbar_qr = _pyzbar, [ZBarSymbol.QRCODE]
        except Exception as e:                             # noqa: BLE001
            print(f"pyzbar yok, QR overlay kapalı: {e}", file=sys.stderr)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    bridge = None
    if not args.no_bridge:
        bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_image", "image_bridge", args.topic],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4.0)

    state = {"frame": None}
    stats = {"n": 0, "t": time.time(), "fps": 0.0}

    class ViewNode(Node):
        def __init__(self):
            super().__init__("front_viewer")
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.topic, self.on_image, qos)

        def on_image(self, msg: Image) -> None:
            rgb = to_array(msg)
            gray = to_gray(rgb)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            tags = qrs = 0
            if det is not None:
                corners, ids, _ = det.detectMarkers(gray)
                tags = draw_tags(bgr, corners, ids)
            if pyzbar is not None:
                qrs = draw_qrs(bgr, pyzbar.decode(gray, symbols=zbar_qr))
            now = time.time()
            stats["fps"] = 0.9 * stats["fps"] + 0.1 / max(now - stats["t"], 1e-3)
            stats["t"] = now
            stats["n"] += 1
            hud = (f"kare {stats['n']}  raf tag: {tags}  kutu QR: {qrs}  "
                   f"{stats['fps']:4.1f} FPS")
            cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(bgr, hud, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            if args.scale != 1.0:
                bgr = cv2.resize(bgr, None, fx=args.scale, fy=args.scale)
            state["frame"] = bgr

    rclpy.init()
    node = ViewNode()
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    print(f"dinleniyor: {args.topic}  (pencereyi kapatmak: q / ESC)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["frame"] is not None:
                cv2.imshow(WIN, state["frame"])
                # pencere-kapatma kontrolü ANCAK imshow bir kez çağrıldıktan
                # (pencere gerçekten oluştuktan) sonra güvenli; yoksa Qt'de
                # "NULL guiReceiver" hatası verir.
                if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
