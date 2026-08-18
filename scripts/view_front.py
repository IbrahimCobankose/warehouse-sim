#!/usr/bin/env python3
"""Ön kamera CANLI PENCERE + işleme overlay'i.

Uçuş sırasında ön kameranın gördüğünü ve kodun o kare üzerinde yaptığı
İŞLEMEYİ ayrı bir pencerede canlı gösterir:
  * RAF AprilTag tespiti (aruco, apriltag_localize.py'daki lokalizasyon
    dedektörünün aynısı) -> yeşil çerçeve + tag id.
  * KUTU QR tespiti (pyzbar, scan_boxes.py'daki tarayıcının aynısı) -> sarı
    poligon + yük metni.
  * KUTU BARKODU (Code128, scan_boxes --with-barcode ile aynı sembol) ->
    macenta çerçeve + yük metni. Kapatmak: --no-barcode.

BARKOD ÇERÇEVESİ ÖLÇÜLDÜ, ÇİZİLMEDİ: zbar linear semboller için KUTU
DÖNDÜRMÜYOR -- rect'in genişliği 0, poligon yalnız çubukların BAŞ KENARI
(iki nokta). Çizilecek bir kutu yok, üretmek gerekiyor. Üretim scan_boxes'ın
zaten doğrulanmış geometrisiyle yapılıyor (BarcodeLinker: barkod QR'ın tam
altında, gen_labels.placard_geometry çubuk ölçüsünü verir) -- yani çerçeve
"bağlandığı QR'a göre çubukların olması gereken yer". Bağlanamayan barkod
için uydurma kutu çizilmiyor: zbar'ın gerçekten gördüğü baş kenarı çizilip
yük "?" ile işaretleniyor.

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
WIN = ("on kamera (canli) -- yesil: raf AprilTag  sari: kutu QR  "
       "macenta: kutu barkodu")


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


def zbar_poly(r) -> np.ndarray:
    """zbar sonucunun köşeleri; poligon hiç yoksa rect'e düş.

    DİKKAT: eşik 2, 3 değil. Code128'de poligon çoğu zaman İKİ nokta (baş
    kenar) ve bu iki nokta gerçek bilgidir -- dikdörtgene çevrilirse hem
    sıfır genişlikli bir kutu çıkar hem de BarcodeLinker okumayı "tam
    poligon" sanıp dejenere okumaya tanıdığı serbestliği uygulamaz."""
    if r.polygon and len(r.polygon) >= 2:
        return np.array([[p.x, p.y] for p in r.polygon], dtype=np.int32)
    x, y, w, h = r.rect.left, r.rect.top, r.rect.width, r.rect.height
    return np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
                    dtype=np.int32)


def draw_qrs(bgr, results, fs, thick) -> int:
    """pyzbar QR sonuçlarını çiz (scan_boxes'ın gördüğü aynı tespit)."""
    n = 0
    for r in results:
        if r.type != "QRCODE":
            continue
        n += 1
        pts = zbar_poly(r)
        cv2.polylines(bgr, [pts], True, (0, 220, 255), thick, cv2.LINE_AA)
        payload = r.data.decode("utf-8", "replace")
        top = pts[pts[:, 1].argmin()]
        label(bgr, payload, (int(top[0]), int(top[1]) - 6),
              (0, 220, 255), fs, thick)
    return n


def draw_barcodes(bgr, results, qrs, linker, bar_wh, fs, thick) -> tuple[int, int]:
    """Code128 sonuçlarını çiz. `(bağlı, bağsız)` döndürür.

    Yük ETİKETİN ALTINA yazılıyor: barkod QR'ın hemen altındaki şeritte
    duruyor, üste yazılsa QR'ın kendi etiketiyle çakışırdı. Renk QR'dan (sarı)
    ve tag'den (yeşil) ayrı olsun diye macenta -- hangi sembolün çözüldüğü tek
    bakışta belli olmalı.
    """
    linked = loose = 0
    bar_w, bar_h = bar_wh
    for r in results:
        if r.type != "CODE128":
            continue
        pts = zbar_poly(r)
        payload = r.data.decode("utf-8", "replace")
        pred = None
        if linker is not None:
            qr_payload = linker.link(qrs, pts.tolist())
            if qr_payload is not None:
                pred = linker.predict(dict(qrs)[qr_payload])
        if pred is not None:                    # QR'a bağlandı -> gerçek çerçeve
            linked += 1
            cx, cy, ppm = pred
            hw, hh = bar_w * ppm / 2.0, bar_h * ppm / 2.0
            p0 = (int(cx - hw), int(cy - hh))
            p1 = (int(cx + hw), int(cy + hh))
            cv2.rectangle(bgr, p0, p1, (255, 0, 255), thick, cv2.LINE_AA)
            label(bgr, payload, (p0[0], p1[1] + int(fs * 26)),
                  (255, 0, 255), fs, thick)
        else:                                   # bağlanamadı -> yalnız ham okuma
            loose += 1
            cv2.polylines(bgr, [pts], False, (255, 0, 255),
                          max(1, thick // 2), cv2.LINE_AA)
            bot = pts[pts[:, 1].argmax()]
            label(bgr, f"{payload} ?", (int(pts[:, 0].min()),
                                        int(bot[1]) + int(fs * 26)),
                  (255, 0, 255), fs, thick)
    return linked, loose


def make_linker(config_path: Path):
    """`(BarcodeLinker, (çubuk_genişliği_m, çubuk_yüksekliği_m))` ya da
    `(None, ...)`. Geometri scan_boxes/gen_labels'tan ALINIYOR, burada
    yeniden türetilmiyor: bu projede etiket geometrisini iki yerde ayrı
    tutmak üç ayrı sessiz kayma hatasına yol açtı."""
    try:
        import yaml
        import gen_labels as gl
        from scan_boxes import BarcodeLinker
        cfg = yaml.safe_load(config_path.read_text())
        codes = cfg["codes"]
        side, rise = gl.box_label_geometry(codes["box_label"],
                                           codes["texture_px_per_m"],
                                           codes["max_texture_px"])
        bar_w, bar_h, _ = gl.placard_geometry(codes["box_placard"],
                                              codes["texture_px_per_m"],
                                              codes["max_texture_px"])
        return BarcodeLinker(cfg, side, rise), (bar_w, bar_h)
    except Exception as e:                                 # noqa: BLE001
        print(f"barkod geometrisi yüklenemedi, çerçeve yerine ham okuma "
              f"çizilecek: {e}", file=sys.stderr)
        return None, (0.0, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=FRONT_TOPIC)
    ap.add_argument("--no-bridge", action="store_true",
                    help="ros_gz_image köprüsü açma (apriltag/scan_boxes açtıysa)")
    ap.add_argument("--no-tags", action="store_true", help="raf AprilTag overlay kapat")
    ap.add_argument("--no-qr", action="store_true", help="kutu QR overlay kapat")
    ap.add_argument("--no-barcode", action="store_true",
                    help="kutu barkodu (Code128) overlay kapat. Açık olması "
                         "ÖLÇÜLDÜ: tam karede Code128 aramak +0.4 ms/kare.")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml",
                    help="barkod çerçevesinin geometrisi buradan okunur")
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

    pyzbar, zbar_syms = None, []
    if not (args.no_qr and args.no_barcode):
        try:
            from pyzbar import pyzbar as _pyzbar
            from pyzbar.pyzbar import ZBarSymbol
            pyzbar = _pyzbar
            if not args.no_qr:
                zbar_syms.append(ZBarSymbol.QRCODE)
            if not args.no_barcode:
                zbar_syms.append(ZBarSymbol.CODE128)
        except Exception as e:                             # noqa: BLE001
            print(f"pyzbar yok, QR/barkod overlay kapalı: {e}", file=sys.stderr)

    # Barkod çerçevesi QR'a bağlanarak çiziliyor -> QR overlay kapalıysa
    # bağlama da yapılamaz (tespit yine görünür, "?" ile).
    linker, bar_wh = (None, (0.0, 0.0))
    if pyzbar is not None and not args.no_barcode:
        if args.no_qr:
            print("--no-qr ile barkod QR'a bağlanamaz: ham okuma çizilecek.",
                  file=sys.stderr)
        else:
            linker, bar_wh = make_linker(args.config)

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
                qr = pyzbar.decode(gray, symbols=zbar_syms)
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
                nq = nb = nloose = 0
                if pyzbar is not None:
                    if not args.no_qr:
                        nq = draw_qrs(bgr, qr, fs, thick)
                    if not args.no_barcode:
                        qrs = [(r.data.decode("utf-8", "replace"),
                                zbar_poly(r).tolist())
                               for r in qr if r.type == "QRCODE"]
                        nb, nloose = draw_barcodes(bgr, qr, qrs, linker,
                                                   bar_wh, fs, thick)
                now = time.time()
                disp["fps"] = 0.9 * disp["fps"] + 0.1 / max(now - disp["t"], 1e-3)
                disp["t"] = now
                far_txt = f" (+{nfar} uzak)" if nfar else ""
                bc_txt = ("" if args.no_barcode else
                          f"  barkod: {nb}" +
                          (f" (+{nloose} bagsiz)" if nloose else ""))
                hud = (f"kare {cap_n}  raf tag: {nt}{far_txt}  kutu QR: {nq}"
                       f"{bc_txt}  {disp['fps']:4.1f} FPS  (tespit {det_n})")
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
