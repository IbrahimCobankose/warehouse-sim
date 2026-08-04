#!/usr/bin/env python3
"""Kutu tarama düğümü: ön kamerayı izler, gördüğü her kutu QR'ının en iyi
karesini diske kaydeder.

Otonom navigasyon yol haritasının 1. aşaması. Navigasyondan bağımsız: aracın
oraya nasıl geldiği önemli değil, düğüm sadece kamera akışını dinliyor.
Bu yüzden bugünkü test uçuşunun askıda bekleme aşamasında bile denenebilir.

    source /opt/ros/jazzy/setup.bash
    .venv/bin/python scripts/scan_boxes.py

VENV PYTHON'U İLE ÇALIŞIR ama ROS source edilmiş olmalı: pyzbar sadece
venv'de, rclpy sadece ROS kurulumunda. İkisi de python 3.12 olduğu için
venv yorumlayıcısı ROS'un site-packages'ını sorunsuz alıyor.

Simülasyon olmadan, kaydedilmiş karelerle mantığı denemek için:

    .venv/bin/python scripts/scan_boxes.py --replay out/frames/probe_box_qr

HIZ SINIRI (ölçüm, 1920x1080, bu makine):
    sadece QR      70-215 ms/kare   ->  5-14 Hz
    QR + Code128   ~750 ms/kare     ->  ~1.3 Hz
Kamera 20 Hz yayınlıyor, yani kareler zaten atlanacak; QoS derinliği 1
olduğu için hep EN YENİ kare işlenir, kuyrukta bayatlamış kare birikmez.
Barkod bu yüzden varsayılan olarak kapalı (--with-barcode ile açılır).
Kareyi küçültmek bir seçenek değil: okunabilirlik bütçesi 1080p'de zaten
4.42 px/modül, yarıya indirince 3'lük eşiğin altına düşer.

Bu sınır rota hızını da bağlıyor: ön kamera 1.5 m'de 1.73 m genişlik görüyor,
kodun kenarda değil ortada olması gerektiği düşünülürse kutu başına ~1 m
kullanılabilir pencere var. Kutu başına 3 deneme istiyorsak seyir hızı
5 Hz x 1 m / 3 ~= 1.5 m/s'yi aşmamalı (4. ve 5. aşama için not).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from ros_image import to_array, to_gray          # noqa: E402

DEFAULT_TOPIC = "/warehouse_scout/camera_front/image"

sys.stdout.reconfigure(line_buffering=True)


def polygon_area(points) -> float:
    """Kabuk (shoelace) formülü. QR'ın karedeki piksel alanı, "bu görüntü ne
    kadar iyi" için en ucuz vekil ölçü: büyük alan = yakın ve karşıdan."""
    n = len(points)
    if n < 3:
        return 0.0
    a = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def safe_name(payload: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", payload).strip("_")[:80]


@dataclass
class Scan:
    """Bir kutu için tutulan kayıt. Dosyaya yazılan biçim budur."""
    seq: int
    payload: str
    image: str
    known: bool
    first_seen_t: float
    first_seen_wall: str
    best_t: float
    best_area_px: float
    best_polygon: list
    sightings: int = 1
    barcodes: list = field(default_factory=list)
    truth: dict | None = None


class Scanner:
    """ROS'tan bağımsız çekirdek: bir kare al, yeni kutuları kaydet.

    Düğüm de replay modu da bunu çağırır; böylece mantık simülasyon olmadan
    test edilebiliyor.
    """

    def __init__(self, out: Path, truth: dict, relock: float,
                 with_barcode: bool, improve_margin: float = 1.05,
                 verbose: bool = False):
        from pyzbar import pyzbar
        from pyzbar.pyzbar import ZBarSymbol

        self._pyzbar = pyzbar
        self.symbols = [ZBarSymbol.QRCODE]
        if with_barcode:
            self.symbols.append(ZBarSymbol.CODE128)

        self.out = out
        self.frames_dir = out / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.records: dict[str, Scan] = {}
        self.last_seen: dict[str, float] = {}
        self.truth = truth
        self.relock = relock
        self.improve_margin = improve_margin
        self.verbose = verbose

        self.frames_processed = 0
        self.decode_time = 0.0
        self.unknown: set[str] = set()

    # ------------------------------------------------------------------ çekirdek
    def process(self, rgb: np.ndarray, t: float) -> list[str]:
        """Bir kareyi işler, YENİ bulunan kutuların yüklerini döndürür."""
        gray = to_gray(rgb)
        t0 = time.perf_counter()
        results = self._pyzbar.decode(gray, symbols=self.symbols)
        self.decode_time += time.perf_counter() - t0
        self.frames_processed += 1

        qrs, barcodes = [], []
        for r in results:
            payload = r.data.decode("utf-8", "replace")
            if r.type == "QRCODE":
                qrs.append((payload, r))
            else:
                # Barkod yükü benzersiz DEĞİL (aynı gözdeki bütün kutular aynı
                # konum kodunu taşıyor), o yüzden asla dedup anahtarı olamaz --
                # sadece karede ne görüldüğünün notu olarak taşınıyor.
                barcodes.append(payload)

        fresh = []
        for payload, r in qrs:
            poly = [[float(p.x), float(p.y)] for p in r.polygon] or [
                [float(r.rect.left), float(r.rect.top)],
                [float(r.rect.left + r.rect.width), float(r.rect.top)],
                [float(r.rect.left + r.rect.width), float(r.rect.top + r.rect.height)],
                [float(r.rect.left), float(r.rect.top + r.rect.height)],
            ]
            area = polygon_area(poly)
            rec = self.records.get(payload)

            if rec is None:
                rec = self._new_record(payload, rgb, t, area, poly, barcodes)
                fresh.append(payload)
            else:
                rec.sightings += 1
                still_visible = (t - self.last_seen.get(payload, t)) <= self.relock
                better = area > rec.best_area_px * self.improve_margin
                if still_visible and better:
                    # Aynı geçişte daha iyi bir açı yakalandı: kareyi güncelle.
                    # Geçiş bittikten (relock penceresi kapandıktan) sonra
                    # gelen tekrar görüşler yok sayılır -- README'nin "aynı
                    # kutuyu tekrar kaydetme" kuralı.
                    self._save_frame(rgb, rec.image)
                    rec.best_area_px, rec.best_polygon, rec.best_t = area, poly, t
                for b in barcodes:
                    if b not in rec.barcodes:
                        rec.barcodes.append(b)
            self.last_seen[payload] = t

        if fresh or (self.verbose and qrs):
            self._write()
        return fresh

    def _new_record(self, payload, rgb, t, area, poly, barcodes) -> Scan:
        seq = len(self.records) + 1
        name = f"{seq:04d}_{safe_name(payload)}.png"
        self._save_frame(rgb, name)

        info = self.truth.get(payload)
        if info is None:
            self.unknown.add(payload)
        rec = Scan(
            seq=seq,
            payload=payload,
            image=f"frames/{name}",
            known=info is not None,
            first_seen_t=round(t, 3),
            first_seen_wall=time.strftime("%Y-%m-%dT%H:%M:%S"),
            best_t=round(t, 3),
            best_area_px=round(area, 1),
            best_polygon=poly,
            barcodes=list(dict.fromkeys(barcodes)),
            truth=None if info is None else {
                "entity": info["entity"], "row": info["row"],
                "bay": info["bay"], "level": info["level"],
                "label_pose_xyzrpy": info["label_pose_xyzrpy"],
            },
        )
        self.records[payload] = rec
        return rec

    def _save_frame(self, rgb: np.ndarray, name: str) -> None:
        PILImage.fromarray(rgb).save(self.frames_dir / name)

    def _write(self) -> None:
        """Kayıtları her değişiklikte diske yazar. Uçuş yarıda kesilse de
        (Ctrl-C, çakılma) o ana kadar taranan kutular kaybolmasın diye;
        atomik değişimle yarım dosya bırakmadan."""
        data = {
            "topic_frames_processed": self.frames_processed,
            "boxes": [asdict(r) for r in
                      sorted(self.records.values(), key=lambda r: r.seq)],
        }
        tmp = self.out / "scans.json.tmp"
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        tmp.replace(self.out / "scans.json")

    # ------------------------------------------------------------------ rapor
    def summary(self) -> None:
        self._write()
        n = self.frames_processed
        ms = 1000 * self.decode_time / n if n else 0.0
        print()
        print(f"işlenen kare      : {n}")
        print(f"çözme süresi      : {ms:.0f} ms/kare "
              f"(üst sınır ~{1000/ms:.1f} Hz)" if n else "")
        print(f"taranan kutu      : {len(self.records)}")
        if self.truth:
            total = sum(1 for c in self.truth.values() if c["type"] == "box_qr")
            print(f"ground truth'ta   : {sum(1 for r in self.records.values() if r.known)}"
                  f"/{total} kutu QR'ı")
        if self.unknown:
            # Ground truth'ta olmayan bir yük, dokunun yanlış render edildiğine
            # (örn. aynalanmış) işaret eder; sessizce geçilmemeli.
            print(f"\nUYARI: ground truth'ta olmayan {len(self.unknown)} yük çözüldü:")
            for p in sorted(self.unknown)[:10]:
                print(f"  {p!r}")
        print(f"\nçıktı: {self.out}")


# ---------------------------------------------------------------------- ROS modu
def run_ros(args, scanner: Scanner) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    class ScanNode(Node):
        def __init__(self):
            super().__init__("warehouse_box_scanner")
            # Kamera yayınları best-effort; varsayılan RELIABLE ile abonelik
            # eşleşmez ve sessizce hiç kare gelmez.
            # Derinlik 1: çözme kamera hızından yavaş, kuyrukta bekleyen eski
            # kareleri işlemenin anlamı yok -- hep en yenisi gelsin.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.topic, self.on_image, qos)
            self.first = True
            self.no_stamp_warned = False
            self.last_status = 0.0

        def on_image(self, msg: Image) -> None:
            if self.first:
                self.first = False
                print(f"ilk kare: {msg.width}x{msg.height} {msg.encoding}")
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0.0:
                # Köprü damgayı doldurmuyorsa duvar saatine düş. Sim zamanı
                # duvar saatiyle aynı hızda akmayabilir (RTF ~0.98), bu yüzden
                # hangisinin kullanıldığı kayıtta belli olmalı.
                if not self.no_stamp_warned:
                    self.no_stamp_warned = True
                    print("UYARI: kare damgası boş, duvar saati kullanılıyor")
                stamp = time.time()
            rgb = to_array(msg)
            for payload in scanner.process(rgb, stamp):
                rec = scanner.records[payload]
                where = ("bilinmiyor" if rec.truth is None else
                         f"{rec.truth['row']}-{rec.truth['bay']:02d}-{rec.truth['level']}")
                print(f"[{rec.seq:3d}] {payload:<26} {where:<12} "
                      f"{rec.best_area_px:6.0f} px²")

            now = time.time()
            if now - self.last_status >= 5.0:
                self.last_status = now
                n = scanner.frames_processed
                ms = 1000 * scanner.decode_time / n if n else 0
                print(f"      ... {n} kare, {len(scanner.records)} kutu, "
                      f"{ms:.0f} ms/kare")

    bridge = None
    if not args.no_bridge:
        # grab_frames.py ile aynı köprü; ikisini aynı anda çalıştırırsanız
        # konuya iki yayıncı olur (zararsız ama gereksiz), birinde --no-bridge
        # kullanın.
        bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_image", "image_bridge", args.topic],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4.0)

    rclpy.init()
    node = ScanNode()
    print(f"dinleniyor: {args.topic}   (durdurmak için Ctrl-C)")
    deadline = time.time() + args.duration if args.duration else None
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    try:
        while rclpy.ok() and not stop["now"]:
            if deadline and time.time() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()

    if scanner.frames_processed == 0:
        print(f"\nHiç kare gelmedi. Simülasyon çalışıyor mu, '{args.topic}' doğru mu?",
              file=sys.stderr)
        return 1
    return 0


# ------------------------------------------------------------------- replay modu
def run_replay(args, scanner: Scanner) -> int:
    """Kaydedilmiş PNG'leri kare kare besler. Simülasyon ve ROS gerektirmez;
    dedup/en-iyi-kare mantığını tekrarlanabilir biçimde test etmek için."""
    files = sorted(p for p in args.replay.iterdir()
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    if not files:
        print(f"{args.replay} içinde kare yok.", file=sys.stderr)
        return 1
    dt = 1.0 / args.replay_hz
    for i, f in enumerate(files):
        rgb = np.asarray(PILImage.open(f).convert("RGB"))
        for payload in scanner.process(rgb, i * dt):
            rec = scanner.records[payload]
            print(f"[{rec.seq:3d}] {f.name:<12} {payload:<26} "
                  f"{rec.best_area_px:6.0f} px²")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "out" / "scans")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--with-barcode", action="store_true",
                    help="Code128'i de çöz (çözme süresini ~3x artırır)")
    ap.add_argument("--relock", type=float, default=3.0,
                    help="bir kutu bu kadar saniye görünmezse geçiş bitti sayılır; "
                         "sonraki görüşler yeniden kaydedilmez")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="bu kadar saniye sonra kendiliğinden dur (0 = süresiz)")
    ap.add_argument("--no-bridge", action="store_true",
                    help="ros_gz köprüsünü başlatma (dışarıda çalışıyorsa)")
    ap.add_argument("--replay", type=Path,
                    help="ROS yerine bu dizindeki PNG'leri işle (offline test)")
    ap.add_argument("--replay-hz", type=float, default=20.0,
                    help="replay'de karelere atanan sanal kare hızı")
    args = ap.parse_args()

    truth = {}
    if args.ground_truth.exists():
        codes = json.loads(args.ground_truth.read_text())["codes"]
        # SADECE kutu QR'ları anahtarlanıyor: barkod yükleri benzersiz değil
        # (120 etiket, 59 farklı yük), hepsini tek sözlüğe koymak kayıtları
        # birbirinin üstüne yazardı.
        truth = {c["payload"]: c for c in codes if c["type"] == "box_qr"}
    else:
        print(f"UYARI: ground truth yok ({args.ground_truth}), doğrulama atlanıyor")

    args.out.mkdir(parents=True, exist_ok=True)
    scanner = Scanner(args.out, truth, args.relock, args.with_barcode)

    rc = run_replay(args, scanner) if args.replay else run_ros(args, scanner)
    scanner.summary()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
