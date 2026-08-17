#!/usr/bin/env python3
"""Envanter kaydı üretici: ham okumalar + görüş pozu -> inventory.json / .csv

    okumalar (out/scans/readings.jsonl)      "karede şu QR şu köşelerdeydi"
  + görüş pozu (out/logs/ev_feed.csv)        "o an araç şuradaydı"
  + kamera modeli (config/warehouse.yaml)
  = ürünün DÜNYA konumu

Uçuş/ROS gerektirmez, üç dosyayı okur:

    .venv/bin/python tools/build_inventory.py
    .venv/bin/python tools/build_inventory.py --truth      # doğruluk ölçümü

NASIL: QR'ın karedeki dört köşesi + kodun GERÇEK fiziksel kenarı biliniyor ->
solvePnP kodun kameraya göre konumunu verir (`tvec`). Araç pozu ve kamera
montajı biliniyor -> konum dünyaya taşınır. Okumalar ürün başına toplanıp
MEDYAN alınır: bir kutu tipik olarak 14 kez okunuyor, tek okumaya güvenmek
gereksiz.

SADECE tvec KULLANILIYOR, rvec KULLANILMIYOR. Düzlemsel dört noktadan PnP'nin
iki çözümü vardır (IPPE düzlem-flip); bu projede flip daha önce lokalizasyonu
5 m yanıltıp aracı rafa çarptırmıştı. Flip belirsizliği YÖNELİMİ bozar,
translasyonu neredeyse hiç etkilemez -- yönelime ihtiyacımız olmadığı için
belirsizliği tamamen dışarıda bırakıyoruz.

İKİ SİSTEMATİK ÖLÇÜ DÜZELTMESİ (ölçüldü, bkz. qr_geometry):
  1. Render edilen QR kenarı config'teki 0.100 m DEĞİL 0.1042 m -- modül
     pikseli tam sayıya yuvarlanıyor (9.6 -> 10 px). gen_labels'ın
     floor_marker_geometry'si aynı tuzağı floor tag için çözüyor.
  2. zbar'ın döndürdüğü poligon gerçek modül sınırının %98.80'i (20 dokuda
     birebir sabit).
İkisi ihmal edilirse mesafe ~%2.9 hatalı, 1.5 m'de ~4.4 cm sistematik kayma.

GROUND TRUTH KULLANILMIYOR (--truth hariç, o da yalnızca ÖLÇÜM için). Raf/
seviye ataması depo YERLEŞİMİNDEN (config/warehouse.yaml) yapılıyor: harita
bilgisi meşru girdi, envanterin kendisi değil.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
import gen_labels as gl                                          # noqa: E402
from apriltag_localize import FRONT_BODY_FROM_CAM, intrinsics    # noqa: E402

def qr_geometry(cfg: dict) -> tuple[float, float]:
    """Kutu QR'ının `(etkin_kenar_m, merkez_yükseklik_ofseti_m)`.

    Hesap gen_labels'ta: etiketi ÇİZEN kodla aynı yerde durması gerekiyor,
    yoksa doku üretimi değişince burası sessizce yanlış kalır.
    """
    codes = cfg["codes"]
    return gl.box_label_geometry(codes["box_label"], codes["texture_px_per_m"],
                                 codes["max_texture_px"])


class PoseTrack:
    """ev_feed.csv -> herhangi bir duvar-saati anında araç pozu (dünya ENU).

    Doğrusal aradeğerleme; yaw en kısa yoldan. Komşu fix `max_dt`den uzaksa
    None döner -- tag boşluğunda (koridor sonu, açık alan dönüşü) poz
    uydurmak yerine o okumayı düşürmek doğru: uydurulmuş poz envantere
    sessizce yanlış konum yazar.
    """

    def __init__(self, path: Path, max_dt: float = 0.30):
        self.max_dt = max_dt
        self.t: list[float] = []
        rows = []
        with path.open() as f:
            for r in csv.DictReader(f):
                rows.append((int(r["wall_ms"]) / 1000.0, float(r["x"]),
                             float(r["y"]), float(r["z"]),
                             math.radians(float(r["yaw_deg"]))))
        rows.sort()
        self.t = [r[0] for r in rows]
        self.p = np.array([[r[1], r[2], r[3]] for r in rows], dtype=np.float64)
        self.yaw = np.array([r[4] for r in rows], dtype=np.float64)

    def __len__(self) -> int:
        return len(self.t)

    def at(self, t: float):
        """(pos(3,), yaw) ya da None."""
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return None if self.t[0] - t > self.max_dt else (self.p[0], self.yaw[0])
        if i >= len(self.t):
            return None if t - self.t[-1] > self.max_dt else (self.p[-1], self.yaw[-1])
        t0, t1 = self.t[i - 1], self.t[i]
        if t - t0 > self.max_dt and t1 - t > self.max_dt:
            return None                       # iki fix arasında büyük boşluk
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        pos = self.p[i - 1] + f * (self.p[i] - self.p[i - 1])
        d = (self.yaw[i] - self.yaw[i - 1] + math.pi) % (2 * math.pi) - math.pi
        return pos, self.yaw[i - 1] + f * d


def order_corners(poly: list) -> np.ndarray | None:
    """Dört köşeyi saat yönünde ve sol-üstten başlayarak sırala.

    zbar'ın köşe sırası sembole göre değişebiliyor; solvePnP'ye yanlış sırada
    verilen köşeler sessizce döndürülmüş/aynalanmış bir çözüm üretir. Sıra
    geometriden yeniden kuruluyor, kaynağa güvenilmiyor. (Görüntü çerçevesinde
    y aşağı olduğu için artan açı = görsel olarak saat yönü.)
    """
    p = np.asarray(poly, dtype=np.float64)
    if p.shape != (4, 2):
        return None
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    p = p[np.argsort(ang)]
    start = int(np.argmin(p[:, 0] + p[:, 1]))        # sol-üste en yakın köşe
    return np.roll(p, -start, axis=0)


def object_points(side: float) -> np.ndarray:
    """IPPE_SQUARE'in beklediği sıra: sol-üst, sağ-üst, sağ-alt, sol-alt."""
    h = side / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float64)


def solve_reading(poly, side: float, K, dist):
    """Bir okumadan kodun KAMERA çerçevesindeki konumu. (t_cam(3,), reproj) ya da None."""
    ip = order_corners(poly)
    if ip is None:
        return None
    obj = object_points(side)
    ok, rvec, tvec = cv2.solvePnP(obj, ip, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rep = float(np.linalg.norm(proj.reshape(4, 2) - ip, axis=1).mean())
    return tvec.ravel(), rep


def to_world(p_body, yaw: float, t_cam, cam_offset) -> np.ndarray:
    """Kamera çerçevesindeki noktayı dünyaya taşı.

    Araç seviye uçtuğu varsayımıyla yalnız yaw kullanılıyor (roll/pitch
    ev_feed'de yok). Tarama bacaklarında eğim birkaç dereceyi geçmiyor;
    1.5 m mesafede 3° ~ 8 cm -- toplam bütçede kabul edilebilir, ama doğruluk
    raporunda beklenmedik bir sapma görülürse ilk şüpheli burasıdır.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.asarray(p_body) + R_wb @ (np.asarray(cam_offset)
                                        + FRONT_BODY_FROM_CAM @ np.asarray(t_cam))


class Layout:
    """Depo yerleşimi: konumdan raf yüzü / seviye / göz türetir.

    Bu HARİTA bilgisi (config/warehouse.yaml), envanter bilgisi değil: hangi
    rafın nerede olduğunu bilmek meşru, hangi ürünün orada olduğunu bilmek
    değil. Ground truth'a dokunulmuyor.
    """

    def __init__(self, cfg: dict):
        r = cfg["racking"]
        self.faces = {}
        for row in r["rows"]:
            # facing +1: ürün yüzü blok bloğun +Y kenarında, -1: -Y kenarında
            y = row["y0"] + r["depth"] if row["facing"] > 0 else row["y0"]
            self.faces[row["id"]] = y
        self.levels = list(r["level_heights"])
        self.bay_width = r["bay_width"]
        self.bay_count = r["bay_count"]
        self.x0 = r["x_origin"]
        # Etiket kirişin ~0.19 m üstünde (kutu ön yüzünün ortası; kutu
        # yükseklikleri 0.30-0.45 arası değişiyor). Seviyeler 1.45 m aralıklı,
        # bu yüzden atama bu kabaca ofsete karşı çok toleranslı.
        self.label_rise = 0.19

    def classify(self, p) -> tuple[str, int, int]:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        row = min(self.faces, key=lambda k: abs(self.faces[k] - y))
        level = 1 + int(np.argmin([abs(z - (h + self.label_rise))
                                   for h in self.levels]))
        bay = int(math.floor((x - self.x0) / self.bay_width)) + 1
        return row, level, max(1, min(self.bay_count, bay))


def parse_payload(payload: str) -> tuple[str | None, dict]:
    """'WH1|A|01|1|SKU53825' -> ('SKU53825', {...}). Çözülemezse (None, {})."""
    parts = payload.split("|")
    if len(parts) != 5:
        return None, {}
    try:
        return parts[4], {"warehouse": parts[0], "row": parts[1],
                          "bay": int(parts[2]), "level": int(parts[3])}
    except ValueError:
        return None, {}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--readings", type=Path,
                    default=PROJECT_ROOT / "out" / "scans" / "readings.jsonl")
    ap.add_argument("--ev", type=Path,
                    default=PROJECT_ROOT / "out" / "logs" / "ev_feed.csv")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "out" / "inventory.json")
    ap.add_argument("--csv", type=Path, default=PROJECT_ROOT / "out" / "inventory.csv")
    ap.add_argument("--max-dt", type=float, default=0.30,
                    help="okuma ile en yakın poz fix'i arasındaki azami fark (s)")
    ap.add_argument("--max-reproj", type=float, default=3.0,
                    help="azami yeniden-yansıtma hatası (px)")
    ap.add_argument("--range", type=float, nargs=2, default=[0.4, 3.5],
                    metavar=("MIN", "MAX"), help="kabul edilen mesafe aralığı (m)")
    ap.add_argument("--truth", action="store_true",
                    help="ground_truth ile doğruluk ÖLÇÜMÜ yaz (kestirime girmez)")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    front = next(c for c in cfg["cameras"] if c["name"] == "front")
    K, dist = intrinsics(front)
    cam_offset = np.array(front["pose"][:3], dtype=np.float64)
    side, qr_rise = qr_geometry(cfg)
    layout = Layout(cfg)

    poses = PoseTrack(args.ev, args.max_dt)
    print(f"poz kaydı     : {len(poses)} fix ({args.ev.name})")
    print(f"QR etkin kenar: {side*1000:.2f} mm   ön kamera fx={K[0,0]:.1f} px")
    print(f"QR merkezi etiket merkezinin {qr_rise*1000:.1f} mm üstünde "
          f"(caption şeridi) -- düzeltiliyor")

    # ---------------------------------------------------------- okumaları çöz
    per_item: dict[str, list] = defaultdict(list)
    n_total = n_nopose = n_reproj = n_range = n_bad = 0
    barcodes: dict[str, set] = defaultdict(set)
    barcode_of: dict[str, tuple[str, int]] = {}      # qr -> (barkod, kalite)

    with args.readings.open() as f:
        for line in f:
            r = json.loads(line)
            n_total += 1
            if r["symbology"] != "QRCODE":
                # Barkod yükü benzersiz değil (aynı gözdeki üç kutu aynı konum
                # kodunu taşıyor), o yüzden ancak scan_boxes'ın geometrik olarak
                # bağladığı QR üzerinden bir kutuya iliştirilebilir. Bağsız
                # okuma yalnız "bu konum kodu görüldü" notu olarak kalıyor.
                barcodes[r["payload"]].add(r["wall_ms"])
                qr = r.get("linked_qr")
                if qr:
                    q = int(r.get("quality", 0) or 0)
                    if qr not in barcode_of or q > barcode_of[qr][1]:
                        barcode_of[qr] = (r["payload"], q)
                continue
            pose = poses.at(r["wall_ms"] / 1000.0)
            if pose is None:
                n_nopose += 1
                continue
            sol = solve_reading(r["polygon"], side, K, dist)
            if sol is None:
                n_bad += 1
                continue
            t_cam, rep = sol
            if rep > args.max_reproj:
                n_reproj += 1
                continue
            rng = float(np.linalg.norm(t_cam))
            if not (args.range[0] <= rng <= args.range[1]):
                n_range += 1
                continue
            p_world = to_world(pose[0], pose[1], t_cam, cam_offset)
            # QR merkezi -> ETİKET merkezi (etiketler dik, ofset saf -Z).
            p_world[2] -= qr_rise
            per_item[r["payload"]].append(
                (p_world, rng, rep, r["wall_ms"], r["area_px"], r["camera"]))

    kept = sum(len(v) for v in per_item.values())
    print(f"\nokuma         : {n_total} ham -> {kept} kullanıldı")
    print(f"  düşen       : poz yok {n_nopose}, reproj {n_reproj}, "
          f"mesafe {n_range}, köşe {n_bad}")

    # ------------------------------------------------------------- topla/yaz
    items = []
    n_rejected = 0
    for payload, obs in sorted(per_item.items()):
        P = np.array([o[0] for o in obs])
        pos = np.median(P, axis=0)
        # İki aşamalı medyan: bir geçişte poz kısa süre kayarsa (koridor
        # uçlarında oluyor) o okumalar medyanı da çeker. Medyandan uzak
        # okumaları atıp yeniden medyan almak bunu toparlıyor. Eşik MAD
        # tabanlı ama bir tabanı var: sağlam bir kayıtta yayılım zaten
        # birkaç cm, sabit 0.25 m eşik iyi okumaları kesmesin diye.
        if len(obs) >= 4:
            d = np.linalg.norm(P - pos, axis=1)
            keep = d <= max(0.25, 3.0 * float(np.median(d)))
            if 3 <= keep.sum() < len(obs):
                n_rejected += int(len(obs) - keep.sum())
                obs = [o for o, k in zip(obs, keep) if k]
                P = np.array([o[0] for o in obs])
                pos = np.median(P, axis=0)
        # Yayılım = medyandan uzaklıkların medyanı (aykırıya dayanıklı).
        spread = float(np.median(np.linalg.norm(P - pos, axis=1)))
        product_id, meta = parse_payload(payload)
        row, level, bay = layout.classify(pos)
        n = len(obs)
        # Güven: okuma sayısı (6'da doyar), yayılımın küçüklüğü (10 cm ölçek),
        # yeniden-yansıtma kalitesi. Üçü de 0..1, çarpım. Kalibre edilmiş bir
        # olasılık DEĞİL, sıralama/eşikleme için sağlam bir skor.
        conf = (min(1.0, n / 6.0)
                * (1.0 / (1.0 + spread / 0.10))
                * (1.0 / (1.0 + float(np.mean([o[2] for o in obs])) / 3.0)))
        ts = sorted(o[3] for o in obs)[n // 2]
        bc = barcode_of.get(payload)
        items.append({
            "product_id": product_id,
            "qr": payload,
            "barcode": bc[0] if bc else None,
            "barcode_quality": bc[1] if bc else None,
            "estimated_x": round(float(pos[0]), 3),
            "estimated_y": round(float(pos[1]), 3),
            "estimated_z": round(float(pos[2]), 3),
            "shelf": row,
            "level": level,
            "bay": bay,
            "camera": obs[0][5],
            "confidence": round(float(conf), 3),
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts / 1000)),
            "n_readings": n,
            "spread_m": round(spread, 3),
            "range_m": round(float(np.median([o[1] for o in obs])), 3),
            "readable": product_id is not None,
        })

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(
        {"source": {"readings": str(args.readings), "ev": str(args.ev)},
         "qr_side_m": round(side, 5),
         "count": len(items),
         "items": items}, indent=2, ensure_ascii=False))

    cols = ["product_id", "qr", "barcode", "barcode_quality", "estimated_x",
            "estimated_y", "estimated_z", "shelf", "level", "bay", "camera",
            "confidence", "timestamp", "n_readings", "spread_m", "range_m",
            "readable"]
    with args.csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for it in items:
            w.writerow(it)

    if n_rejected:
        print(f"  kayıt içi aykırı elenen okuma: {n_rejected}")
    unreadable = sum(1 for it in items if not it["readable"])
    print(f"\nenvanter      : {len(items)} kayıt "
          f"({unreadable} çözülemeyen yük)" if unreadable else
          f"\nenvanter      : {len(items)} kayıt")
    print(f"  {args.out}")
    print(f"  {args.csv}")
    if barcodes:
        n_bc = sum(1 for it in items if it["barcode"])
        print(f"  barkod: {n_bc}/{len(items)} kayda bağlandı "
              f"({len(barcodes)} farklı konum kodu görüldü)")

    if args.truth:
        report_accuracy(items, args.ground_truth)
    return 0


def report_accuracy(items: list, gt_path: Path) -> None:
    """Doğruluk ÖLÇÜMÜ. Ground truth yalnızca burada, yalnızca ölçüm için
    okunuyor -- kestirime hiçbir şekilde girmiyor (proje kuralı)."""
    if not gt_path.exists():
        print(f"\nground truth yok: {gt_path}")
        return
    truth = {c["payload"]: c for c in json.loads(gt_path.read_text())["codes"]
             if c["type"] == "box_qr"}
    errs, wrong_shelf, wrong_level, wrong_bay, unmatched = [], 0, 0, 0, 0
    for it in items:
        t = truth.get(it["qr"])
        if t is None:
            unmatched += 1
            continue
        tx, ty, tz = t["label_pose_xyzrpy"][:3]
        errs.append(math.dist((it["estimated_x"], it["estimated_y"],
                               it["estimated_z"]), (tx, ty, tz)))
        wrong_shelf += it["shelf"] != t["row"]
        wrong_level += it["level"] != t["level"]
        wrong_bay += it["bay"] != t["bay"]
    if not errs:
        print("\nölçülecek eşleşme yok.")
        return
    e = sorted(errs)
    print(f"\nDOĞRULUK ÖLÇÜMÜ ({len(e)}/{len(truth)} kutu, ground truth SADECE burada)")
    print(f"  konum hatası : medyan {e[len(e)//2]:.3f} m   "
          f"ort {sum(e)/len(e):.3f} m   P95 {e[int(0.95*len(e))]:.3f} m   "
          f"maks {e[-1]:.3f} m")
    print(f"  yanlış raf   : {wrong_shelf}")
    print(f"  yanlış seviye: {wrong_level}")
    print(f"  yanlış göz   : {wrong_bay}")
    if unmatched:
        print(f"  ground truth'ta olmayan yük: {unmatched}")


if __name__ == "__main__":
    raise SystemExit(main())
