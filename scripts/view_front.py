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
import threading
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


FONT = cv2.FONT_HERSHEY_SIMPLEX


def label(bgr, text, org, color, fs, thick) -> None:
    """Okunur etiket: koyu zemin şeridi + üzerine renkli, kalın yazı."""
    (tw, th), base = cv2.getTextSize(text, FONT, fs, thick)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(bgr, (x - 2, y - th - base - 2), (x + tw + 2, y + base),
                  (0, 0, 0), -1)
    cv2.putText(bgr, text, (x, y - base + 1), FONT, fs, color, thick,
                cv2.LINE_AA)


def draw_tags(bgr, corners, ids, fs, thick, min_px) -> tuple[int, int]:
    """Aruco raf tag'lerini çiz. YAKIN (kullanılan) tag'ler dolu yeşil; UZAK
    (arka koridor) tag'ler soluk gri. Ayrım görünen kenar uzunluğuyla: raf
    tag'i sabit boyutlu, taranan ön yüz ~1.5 m'de büyük görünür, karşı koridor
    yüzü ~5-7 m'de çok küçük. `solve_rack`'in 3 m mesafe kapısının görsel
    karşılığı (view'da poz yok, kenar uzunluğu mesafe vekili). (yakın, uzak)."""
    if ids is None or len(ids) == 0:
        return 0, 0
    near = far = 0
    for c, i in zip(corners, ids.ravel()):
        quad = c.reshape(4, 2).astype(np.float64)
        side = float(np.linalg.norm(quad - np.roll(quad, 1, axis=0),
                                    axis=1).mean())
        pts = quad.astype(np.int32)
        top = pts[pts[:, 1].argmin()]
        if side >= min_px:                             # yakın -> kullanılır
            near += 1
            cv2.polylines(bgr, [pts], True, (0, 255, 0), thick, cv2.LINE_AA)
            label(bgr, f"tag {int(i)}", (top[0] - 6, top[1] - 6),
                  (0, 255, 0), fs, thick)
        else:                                          # uzak arka koridor
            far += 1
            dim = max(1, thick // 2)
            cv2.polylines(bgr, [pts], True, (140, 140, 140), dim, cv2.LINE_AA)
            cv2.putText(bgr, f"{int(i)}", (top[0] - 4, top[1] - 4), FONT,
                        max(0.4, fs * 0.55), (140, 140, 140), dim, cv2.LINE_AA)
    return near, far


def draw_qrs(bgr, results, fs, thick) -> int:
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
        cv2.polylines(bgr, [pts], True, (0, 220, 255), thick, cv2.LINE_AA)
        payload = r.data.decode("utf-8", "replace")
        top = pts[pts[:, 1].argmin()]
        label(bgr, payload, (int(top[0]), int(top[1]) - 6),
              (0, 220, 255), fs, thick)
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=FRONT_TOPIC)
    ap.add_argument("--no-bridge", action="store_true",
                    help="ros_gz_image köprüsü açma (apriltag/scan_boxes açtıysa)")
    ap.add_argument("--no-tags", action="store_true", help="raf AprilTag overlay kapat")
    ap.add_argument("--no-qr", action="store_true", help="kutu QR overlay kapat")
    ap.add_argument("--scale", type=float, default=0.5,
                    help="başlangıç pencere ölçeği (pencere ayrıca fareyle "
                         "yeniden boyutlandırılabilir)")
    ap.add_argument("--min-tag-px", type=int, default=0, metavar="PX",
                    help="raf tag'i kenarı bu pikselden küçükse UZAK (arka "
                         "koridor) sayılıp soluk çizilir. 0 = otomatik "
                         "(kare yüksekliği x 0.045)")
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

    # Ham kare akışı (callback) ile AĞIR tespit (worker thread) ayrıldı:
    # görüntü ana thread'de takılmadan basılır, overlay biraz gecikmeyle
    # ama düzenli güncellenir. Kilit altında yalnız referanslar takas edilir.
    lock = threading.Lock()
    shared = {"raw": None, "gray": None, "cap_n": 0,
              "tags": (None, None), "qr": [], "det_n": 0}
    stop = threading.Event()
    have_det = det is not None or pyzbar is not None

    class ViewNode(Node):
        def __init__(self):
            super().__init__("front_viewer")
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.topic, self.on_image, qos)

        def on_image(self, msg: Image) -> None:
            # UCUZ: yalnız kareyi sakla; ağır tespit worker'da.
            rgb = to_array(msg)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            gray = to_gray(rgb) if have_det else None
            with lock:
                shared["raw"] = bgr
                shared["gray"] = gray
                shared["cap_n"] += 1

    def detect_worker() -> None:
        last = -1
        while not stop.is_set():
            with lock:
                gray, n = shared["gray"], shared["cap_n"]
            if gray is None or n == last:
                time.sleep(0.002)               # yeni kare yok, boşta bekle
                continue
            last = n
            tags, qr = (None, None), []
            if det is not None:
                corners, ids, _ = det.detectMarkers(gray)
                tags = (corners, ids)
            if pyzbar is not None:
                qr = pyzbar.decode(gray, symbols=zbar_qr)
            with lock:
                shared["tags"], shared["qr"] = tags, qr
                shared["det_n"] += 1

    rclpy.init()
    node = ViewNode()
    # ROS spin'i ayrı thread'de: callback ana GUI döngüsünü bloklamaz.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    worker = (threading.Thread(target=detect_worker, daemon=True)
              if have_det else None)
    if worker:
        worker.start()

    # WINDOW_NORMAL: pencere fareyle serbestçe ölçeklenebilir (AUTOSIZE değil)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    sized = False
    last_shown = -1
    disp = {"t": time.time(), "fps": 0.0}
    print(f"dinleniyor: {args.topic}  (pencereyi kapatmak: q / ESC)")
    try:
        while rclpy.ok() and not stop.is_set():
            with lock:
                raw = shared["raw"]
                bgr = None if raw is None else raw.copy()
                tags, qr = shared["tags"], shared["qr"]
                cap_n, det_n = shared["cap_n"], shared["det_n"]
            if bgr is not None and cap_n != last_shown:
                last_shown = cap_n
                # çizgi/yazı boyutunu kare yüksekliğine göre ölçekle
                h = bgr.shape[0]
                thick = max(2, round(h / 320))
                fs = max(0.6, h / 720.0 * 0.9)
                min_px = args.min_tag_px if args.min_tag_px > 0 else h * 0.045
                nt = nfar = 0
                if det is not None:
                    nt, nfar = draw_tags(bgr, *tags, fs, thick, min_px)
                nq = draw_qrs(bgr, qr, fs, thick) if pyzbar is not None else 0
                now = time.time()
                disp["fps"] = 0.9 * disp["fps"] + 0.1 / max(now - disp["t"], 1e-3)
                disp["t"] = now
                far_txt = f" (+{nfar} uzak)" if nfar else ""
                hud = (f"kare {cap_n}  raf tag: {nt}{far_txt}  kutu QR: {nq}  "
                       f"{disp['fps']:4.1f} FPS  (tespit {det_n})")
                bar_h = int(fs * 34) + 12
                cv2.rectangle(bgr, (0, 0), (bgr.shape[1], bar_h), (0, 0, 0), -1)
                cv2.putText(bgr, hud, (10, bar_h - 12), FONT, fs,
                            (255, 255, 255), thick, cv2.LINE_AA)
                if args.scale != 1.0:
                    bgr = cv2.resize(bgr, None, fx=args.scale, fy=args.scale,
                                     interpolation=cv2.INTER_AREA)
                cv2.imshow(WIN, bgr)
                if not sized:
                    fh, fw = bgr.shape[:2]
                    cv2.resizeWindow(WIN, fw, fh)
                    sized = True
                # pencere-kapatma kontrolü ANCAK imshow bir kez çağrıldıktan
                # (pencere gerçekten oluştuktan) sonra güvenli.
                if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if (cv2.waitKey(15) & 0xFF) in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()               # spin thread'i döndürür
        spin.join(timeout=2)
        if worker:
            worker.join(timeout=2)
        node.destroy_node()
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
