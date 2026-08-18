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
    sadece QR      24-53 ms/kare    ->  19-40 Hz
    QR + Code128   +0.4 ms          ->  pratikte aynı
Kamera 20 Hz yayınlıyor; QoS derinliği 1 olduğu için hep EN YENİ kare işlenir,
kuyrukta bayatlamış kare birikmez. Kareyi küçültmek bir seçenek değil:
okunabilirlik bütçesi 1080p'de zaten 4.42 px/modül, yarıya indirince 3'lük
eşiğin altına düşer.

ESKİ NOT DÜZELTİLDİ (2026-08-14): burada uzun süre "QR + Code128 ~750 ms/kare
-> 1.3 Hz" yazıyordu ve barkod bu yüzden kapalı tutuluyordu. Yeniden ölçüldü:
Code128'i eklemek 24.4 -> 24.8 ms, yani bedava. Asıl darboğaz çözme değil,
callback içinde yapılan 1080p PNG kaydıydı (145 ms) -- bkz. FrameWriter.

Bu sınır rota hızını da bağlıyor: ön kamera 1.5 m'de 1.73 m genişlik görüyor,
kodun kenarda değil ortada olması gerektiği düşünülürse kutu başına ~1 m
kullanılabilir pencere var. Kutu başına 3 deneme istiyorsak seyir hızı
5 Hz x 1 m / 3 ~= 1.5 m/s'yi aşmamalı (4. ve 5. aşama için not).

ÇIKTILAR (out/scans/):
    scans.json    kutu başına ÖZET kayıt + en iyi kare  (eskiden beri; kapsama
                  raporunun girdisi)
    readings.jsonl  HER okuma ayrı satır (poligon, alan, iki saat) -- envanter
                  konumu tek okumadan değil, bir geçişteki okumaların
                  tamamından kestirileceği için özet yetmiyor
    frames.csv    kare düzeyinde teşhis (çözme süresi, kod sayısı, kare
                  aralığı, tahmini düşen kare) -- "QR kaçtı" derken kadraja mı
                  girmedi yoksa kare mi işlenmedi sorusunu ayıran veri
    frames_miss/  hiçbir şey çözülemeyen karelerden örnekler (negatif örnek);
                  decoder iyileştirmesi bunlar üstünde --replay ile ölçülür

İKİ SAAT: her kayıtta hem görüntünün sim zamanı damgası (`t`) hem kareyi
aldığımız andaki duvar saati (`wall_ms`) var. Poz eşlemesi wall_ms üzerinden
yapılır çünkü apriltag_localize'ın ev_feed.csv'si duvar saatiyle yazıyor;
ikisini karıştırmak sessiz bir kayma üretir (bkz. tools/ev_align.py).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from ros_image import to_array, to_gray          # noqa: E402
from gen_world import LABEL_GAP                  # noqa: E402
import gen_labels as gl                          # noqa: E402

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


class FrameWriter:
    """1080p PNG kaydını ROS callback'inden ayıran yazıcı thread'i.

    ÖLÇÜM: bir 1920x1080 karenin PNG'ye yazılması 145 ms; çözme 53 ms. Kayıt
    callback içinde yapılınca düğüm o süre boyunca yeni kare alamıyordu --
    tam turda etkin hız 6.8 Hz'de kalıyor, kare boşlukları 1.05 s'ye çıkıyordu
    (çözme tek başına 18.7 Hz'e izin verdiği hâlde). view_front.py'de aynı
    desen aynı şekilde çözülmüştü: ağır iş callback'i bloklar.

    Kuyruk SINIRLI ve taşınca kare DÜŞÜRÜLÜR: bir kare 6 MB, sınırsız kuyruk
    belleği yer. Düşen kare kayıp değil -- aynı kutu geçiş boyunca defalarca
    görülüyor (medyan 14 okuma), kaydedilen "en iyi kare" biraz daha kötü
    olabilir sadece. Kilitlenen bir düğümün maliyeti bundan çok daha yüksek.
    """

    def __init__(self, max_pending: int = 6):
        self.q: queue.Queue = queue.Queue(maxsize=max_pending)
        self.dropped = 0
        self.written = 0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, name="frame_writer",
                                    daemon=True)
        self._th.start()

    def save(self, path: Path, rgb: np.ndarray) -> bool:
        """Kareyi kuyruğa koyar. Dizi KOPYALANIR: çağıran taraftaki `rgb`
        ROS mesajının tamponuna bakan bir görünüm, mesaj serbest bırakılınca
        altından çekilir. Kopyalama (~6 MB memcpy) birkaç ms, kaydın 145 ms'i
        yanında ihmal edilebilir."""
        try:
            self.q.put_nowait((path, np.ascontiguousarray(rgb)))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                path, arr = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                PILImage.fromarray(arr).save(path)
                self.written += 1
            except Exception as e:                     # disk dolu, izin, vb.
                print(f"UYARI: kare yazılamadı ({path.name}): {e}",
                      file=sys.stderr)

    def close(self, timeout: float = 30.0) -> None:
        """Kuyruktaki kareleri bitirir. Uçuş sonunda birikmiş olabilir; en iyi
        kareler raporun parçası olduğu için beklemeye değer."""
        deadline = time.time() + timeout
        while not self.q.empty() and time.time() < deadline:
            time.sleep(0.05)
        self._stop.set()
        self._th.join(timeout=2.0)


def result_polygon(r) -> list:
    """pyzbar sonucundan köşe listesi. Poligon boşsa (bazı barkodlarda oluyor)
    sınırlayıcı dikdörtgene düşer. Konum kestirimi bu köşelerden yapılacağı
    için kaynağın hangisi olduğu önemli: dikdörtgen perspektifi taşımaz."""
    poly = [[float(p.x), float(p.y)] for p in r.polygon]
    if poly:
        return poly
    return [
        [float(r.rect.left), float(r.rect.top)],
        [float(r.rect.left + r.rect.width), float(r.rect.top)],
        [float(r.rect.left + r.rect.width), float(r.rect.top + r.rect.height)],
        [float(r.rect.left), float(r.rect.top + r.rect.height)],
    ]


class BarcodeLinker:
    """Çözülmüş bir barkodu, ait olduğu kutunun QR'ına bağlar.

    NEDEN GEREKLİ: barkod yükü benzersiz DEĞİL -- aynı gözdeki üç kutu aynı
    konum kodunu (örn. "A0101") taşıyor. Tek başına bir barkod okuması "bu
    hangi kutu" sorusunu cevaplamıyor. Oysa dünya üreticisi barkodu QR
    etiketinin TAM ALTINA, aynı x/y'ye koyuyor (gen_world.inventory:
    pc_z = qr_z - lh/2 - LABEL_GAP - ph/2). QR'ın karedeki köşelerinden piksel
    ölçeği bilindiği için barkodun NEREDE OLMASI GEREKTİĞİ hesaplanabilir;
    gerçekte çözülen barkod bu tahmine en yakın QR'a bağlanır.

    ÜÇ CAPTION DÜZELTMESİ: QR kodu kendi etiketinin merkezinin `qr_rise`
    kadar üstünde, barkod çubukları kendi etiketinin merkezinin `bar_rise`
    kadar üstünde. İkisi de ihmal edilirse tahmin 13-15 mm kayar. (Aynı
    tuzak floor tag'de ve envanter konum kestiriminde de çıkmıştı.)

    ROI DENENDİ VE BIRAKILDI: barkodu yalnız bu tahmini bölgede aramak
    (tam kare yerine) mantıklı görünüyordu ama ölçüm iki gerekçeyi de çürüttü.
    (1) Hız: tam karede Code128 aramak +0.4 ms -- eski nottaki "~750 ms"
    bayat. (2) İsabet: dar kırpma barkodun sessiz alanını yok ediyor (etikette
    yanlarda yalnız 20 mm var, bkz. gen_labels.make_bay_placard yorumu) ve
    ROI 80 karede 15 yük bulurken tam kare 25 buluyor. Geometri bu yüzden
    ARAMAYA değil BAĞLAMAYA hizmet ediyor.

    DEJENERE POLİGON (ÖLÇÜLDÜ): zbar linear sembol için kutu döndürmüyor --
    okumaların ~1/4'ünde poligon İKİ NOKTA (çubukların yalnız baş kenarı),
    rect genişliği 0. Böyle bir okumanın "merkezi" gerçek merkezin yarım
    çubuk genişliği kadar yanında kalıyor: 120 karede ölçülen sapma dx
    medyan +0.135 m, çubuk yarısı 0.148 m ile birebir. Tolerans 0.12 m
    olduğu için bu okumalar DOĞRU QR ile bile HER ZAMAN reddediliyordu.
    Çözüm konumu uydurmak DEĞİL (yön bilinmiyor: baş kenar soldaysa +x,
    sağdaysa -x): eşleşme testi okumanın taşıdığı bilgiye indirgeniyor --
    çubuk ekseni boyunca serbestlik, dikeyde tam tolerans. Tam poligonlu
    okumalarda eski dairesel test aynen kalıyor (orada sapma zaten -2.5 mm).

    VARSAYIM: etiketler dik, araç seviye uçuyor -> karede "aşağı" ~ +y.
    """

    def __init__(self, cfg: dict, qr_side_m: float, qr_rise_m: float,
                 tol_m: float = 0.12):
        codes = cfg["codes"]
        lh = codes["box_label"]["label"][1]
        ph = codes["box_placard"]["label"][1]
        self.bar_w, _, bar_rise = gl.placard_geometry(
            codes["box_placard"], codes["texture_px_per_m"],
            codes["max_texture_px"])
        self.drop_m = qr_rise_m + lh / 2.0 + LABEL_GAP + ph / 2.0 - bar_rise
        self.qr_side_m = qr_side_m
        # Eşleşme toleransı metre cinsinden: kutular ~0.4 m aralıklı, 0.12 m
        # komşu kutunun barkodunu yanlışlıkla bağlamayacak kadar dar.
        self.tol_m = tol_m

    def predict(self, qr_poly: list):
        """QR'dan barkodun beklenen merkezi. `(x, y, px_per_m)` ya da None."""
        p = np.asarray(qr_poly, dtype=np.float64)
        n = len(p)
        side_px = float(np.mean([np.linalg.norm(p[i] - p[(i + 1) % n])
                                 for i in range(n)]))
        if side_px < 12:                   # çok uzak: ölçek güvenilmez
            return None
        ppm = side_px / self.qr_side_m
        return p[:, 0].mean(), p[:, 1].mean() + self.drop_m * ppm, ppm

    def link(self, qrs: list, bar_poly: list) -> str | None:
        """Barkodun okunan yerine en iyi uyan QR yükü.

        Poligon tamsa merkez-merkez mesafesi; dejenere (<=2 nokta) okumada
        yalnız baş kenar bilindiği için çubuk ekseni boyunca yarım çubuk
        kadar serbestlik tanınıyor (bkz. sınıf docstring'i).
        """
        b = np.asarray(bar_poly, dtype=np.float64)
        bx, by = b[:, 0].mean(), b[:, 1].mean()
        edge_only = len(b) <= 2
        slack_x = self.bar_w / 2.0 if edge_only else 0.0
        best, best_d = None, 1e18
        for payload, poly in qrs:
            pred = self.predict(poly)
            if pred is None:
                continue
            px, py, ppm = pred
            dx = abs(bx - px) / ppm                     # metre cinsinden
            dy = abs(by - py) / ppm
            d = math.hypot(max(dx - slack_x, 0.0), dy)
            if d < best_d:
                best, best_d = payload, d
        return best if best_d <= self.tol_m else None


def safe_name(payload: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", payload).strip("_")[:80]


@dataclass
class Scan:
    """Bir kutu için tutulan ÖZET kayıt. scans.json'a yazılan biçim budur.

    Okumaların tamamı burada değil readings.jsonl'da: envanter konumu bir
    geçişteki bütün okumalardan kestirilecek, özet kayıt yalnızca "bu kutu
    görüldü mü" sorusunu (kapsama raporu) cevaplıyor.
    """
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
    # Kare yazıcı kuyruğu doluyken kayıt düşebilir. Düştüyse `image` var
    # olmayan bir dosyayı gösterirdi; bayrak sayesinde sonraki görüşte
    # yeniden denenir ve rapor tutarlı kalır.
    image_saved: bool = True


class Scanner:
    """ROS'tan bağımsız çekirdek: bir kare al, yeni kutuları kaydet.

    Düğüm de replay modu da bunu çağırır; böylece mantık simülasyon olmadan
    test edilebiliyor.
    """

    def __init__(self, out: Path, truth: dict, relock: float,
                 with_barcode: bool, improve_margin: float = 1.05,
                 verbose: bool = False, camera: str = "front",
                 nominal_hz: float = 20.0, miss_every: float = 3.0,
                 miss_max: int = 40, linker: "BarcodeLinker | None" = None):
        from pyzbar import pyzbar
        from pyzbar.pyzbar import ZBarSymbol

        self._pyzbar = pyzbar
        self.symbols = [ZBarSymbol.QRCODE]
        # ÖLÇÜM: Code128'i tam kareye eklemek +0.4 ms (24.4 -> 24.8 ms/kare).
        # Modül başlığındaki eski "~750 ms" notu bu ortamda geçerli değil.
        if with_barcode:
            self.symbols.append(ZBarSymbol.CODE128)
        self.linker = linker if with_barcode else None
        self.barcode_of: dict[str, tuple[str, int]] = {}   # qr -> (barkod, kalite)
        self.bar_seen = 0
        self.bar_linked = 0

        self.out = out
        self.frames_dir = out / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.records: dict[str, Scan] = {}
        self.last_seen: dict[str, float] = {}
        self.truth = truth
        self.relock = relock
        self.improve_margin = improve_margin
        self.verbose = verbose
        self.camera = camera

        self.frames_processed = 0
        self.decode_time = 0.0
        self.unknown: set[str] = set()

        # ---- teşhis katmanı -------------------------------------------------
        # Satır tamponlu açılıyor: uçuş Ctrl-C ile kesilse de (bu projede sık
        # oluyor) o ana kadarki kayıt diskte kalsın.
        self.readings_path = out / "readings.jsonl"
        self._readings = self.readings_path.open("w", buffering=1)
        self.n_readings = 0

        self.frames_csv_path = out / "frames.csv"
        self._frames_csv = self.frames_csv_path.open("w", buffering=1)
        self._frames_csv.write("wall_ms,t_sim,decode_ms,n_qr,n_bar,gap_s,"
                               "est_drops,miss_image\n")

        # Kamera nominal_hz yayınlıyor ama çözme ondan yavaş; ROS (derinlik 1)
        # aradaki kareleri sessizce atıyor. Kaç kare atlandığını doğrudan
        # sayamayız -- ama ardışık kareler arası SİM ZAMANI boşluğundan
        # kestirebiliriz. "QR kadraja girdi ama o an kare işlenmedi" arıza
        # modunu ancak bu sayı görünür kılıyor.
        self.nominal_dt = 1.0 / nominal_hz if nominal_hz > 0 else 0.0
        self.est_drops = 0
        self.first_t: float | None = None
        self.last_t: float | None = None
        self.max_gap_s = 0.0

        # Hiçbir şey çözülemeyen karelerden örnek: decoder iyileştirmesinin
        # (crop/threshold/warp) ölçülebilmesi için NEGATİF örnek şart, yoksa
        # sadece zaten okunan kareler üstünde ayar yapmış oluruz.
        self.miss_dir = out / "frames_miss"
        self.miss_every = miss_every
        self.miss_max = miss_max
        self.misses_saved = 0
        self._last_miss_t = -1e9

        # Kare yazımı ayrı thread'te: 145 ms/kare, callback'i bloklamamalı.
        self.writer = FrameWriter()

    # ------------------------------------------------------------------ çekirdek
    def process(self, rgb: np.ndarray, t: float,
                wall_ms: int | None = None) -> list[str]:
        """Bir kareyi işler, YENİ bulunan kutuların yüklerini döndürür.

        `t` görüntünün sim zamanı damgası, `wall_ms` kareyi aldığımız andaki
        duvar saati (poz eşlemesinin anahtarı; bkz. modül başlığı).
        """
        if wall_ms is None:
            wall_ms = int(time.time() * 1000)
        gray = to_gray(rgb)
        t0 = time.perf_counter()
        results = self._pyzbar.decode(gray, symbols=self.symbols)
        decode_s = time.perf_counter() - t0
        self.decode_time += decode_s
        self.frames_processed += 1

        # Kare aralığı: nominal periyodun üstündeki her boşluk düşen kare.
        gap = 0.0
        if self.first_t is None:
            self.first_t = t
        if self.last_t is not None and self.nominal_dt > 0:
            gap = t - self.last_t
            if gap > 0:
                self.max_gap_s = max(self.max_gap_s, gap)
                # 1.5 payı: 20 Hz yayında damgalar tam periyotta gelmiyor,
                # yuvarlama gürültüsünü düşen kare saymamak için.
                self.est_drops += max(0, int(gap / self.nominal_dt - 1.5))
        self.last_t = t

        qrs, bars = [], []
        for r in results:
            payload = r.data.decode("utf-8", "replace")
            (qrs if r.type == "QRCODE" else bars).append((payload, r))
        barcodes = self._handle_barcodes(qrs, bars, t, wall_ms)

        miss_image = self._maybe_save_miss(rgb, t, wall_ms, results)
        self._frames_csv.write(
            f"{wall_ms},{t:.3f},{1000*decode_s:.1f},{len(qrs)},{len(barcodes)},"
            f"{gap:.3f},{self.est_drops},{miss_image}\n")

        fresh = []
        for payload, r in qrs:
            poly = result_polygon(r)
            area = polygon_area(poly)
            self._log_reading(payload, "QRCODE", poly, area, t, wall_ms)
            rec = self.records.get(payload)

            if rec is None:
                rec = self._new_record(payload, rgb, t, area, poly, barcodes)
                fresh.append(payload)
            else:
                rec.sightings += 1
                still_visible = (t - self.last_seen.get(payload, t)) <= self.relock
                better = area > rec.best_area_px * self.improve_margin
                if not rec.image_saved:
                    # İlk kayıt kuyruk doluyken düşmüştü; kadraj kalitesine
                    # bakmadan yeniden dene -- kaydın karesiz kalmasındansa
                    # biraz daha kötü bir kare yeğdir.
                    rec.image_saved = self._save_frame(rgb, Path(rec.image).name)
                    if rec.image_saved:
                        rec.best_area_px, rec.best_polygon, rec.best_t = area, poly, t
                elif still_visible and better:
                    # Aynı geçişte daha iyi bir açı yakalandı: kareyi güncelle.
                    # Geçiş bittikten (relock penceresi kapandıktan) sonra
                    # gelen tekrar görüşler yok sayılır -- README'nin "aynı
                    # kutuyu tekrar kaydetme" kuralı.
                    # rec.image "frames/xxx.png" (JSON'da göreli yol dursun
                    # diye); _save_frame zaten frames_dir'e yazıyor, tam yolu
                    # vermek frames/frames/... üretiyordu. Sadece askıda
                    # bekleyen testlerde bu dal hiç çalışmadığı için 1.
                    # aşamada fark edilmemişti.
                    self._save_frame(rgb, Path(rec.image).name)
                    rec.best_area_px, rec.best_polygon, rec.best_t = area, poly, t
                for b in barcodes:
                    if b not in rec.barcodes:
                        rec.barcodes.append(b)
            self.last_seen[payload] = t

        if fresh or (self.verbose and qrs):
            self._write()
        return fresh

    # -------------------------------------------------------------- barkod
    def _handle_barcodes(self, qrs: list, bars: list, t: float,
                         wall_ms: int) -> list[str]:
        """Karedeki barkodları kutulara bağlar, ham okuma olarak yazar.

        AYNI BARKODUN TEKRAR OKUNMASINI FİLTRELE: bir kutunun barkodu bir kez
        çözülünce, sonraki okumalar ancak DAHA İYİ kalitede ise kaydı günceller
        (`barcode_of`). Ham okuma akışı yine hepsini tutuyor -- benzersizleştirme
        rapor katmanının işi, çözüm katmanının değil.
        """
        seen_payloads = []
        qr_polys = [(p, result_polygon(r)) for p, r in qrs]
        for payload, r in bars:
            self.bar_seen += 1
            poly = result_polygon(r)
            quality = int(getattr(r, "quality", 0) or 0)
            linked = self.linker.link(qr_polys, poly) if self.linker else None
            if linked:
                self.bar_linked += 1
                best = self.barcode_of.get(linked)
                if best is None or quality > best[1]:
                    self.barcode_of[linked] = (payload, quality)
            self._log_reading(payload, "CODE128", poly, polygon_area(poly),
                              t, wall_ms, quality=quality, linked_qr=linked)
            seen_payloads.append(payload)
        return seen_payloads

    # -------------------------------------------------------------- teşhis
    def _log_reading(self, payload: str, symbology: str, poly: list,
                     area: float, t: float, wall_ms: int,
                     quality: int | None = None,
                     linked_qr: str | None = None) -> None:
        """Bir okumayı readings.jsonl'a yazar. HER okuma yazılır -- aynı kutu
        aynı geçişte on kez okunduysa on satır olur. Envanter konumu bu
        satırların tamamından kestirilecek (tek okumanın konum hatası çok
        daha büyük); benzersizleştirme rapor katmanında yapılıyor."""
        self.n_readings += 1
        rec = {
            "wall_ms": wall_ms,
            "t": round(t, 3),
            "camera": self.camera,
            "symbology": symbology,
            "payload": payload,
            "area_px": round(area, 1),
            "polygon": [[round(x, 1), round(y, 1)] for x, y in poly],
        }
        if quality is not None:
            rec["quality"] = quality
        if linked_qr is not None:
            # Barkod yükü benzersiz değil (aynı gözdeki üç kutu aynı konum
            # kodunu taşıyor); hangi kutuya ait olduğu ancak QR ile bilinir.
            rec["linked_qr"] = linked_qr
        self._readings.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def _maybe_save_miss(self, rgb, t: float, wall_ms: int, results) -> str:
        """Hiçbir kod çözülemeyen kareden örnek saklar (hız sınırlı).

        Boş kare çok -- araç kadrajda kod olmayan yerlerden de geçiyor -- bu
        yüzden hepsini yazmak diski doldurur; amaç temsilî bir negatif küme.
        Dosya adında wall_ms var: sonradan poz eşlemesiyle "bu kare nereye
        bakıyordu, orada bir kutu var mıydı" sorusu cevaplanabilsin diye.
        """
        if results or self.misses_saved >= self.miss_max:
            return ""
        if t - self._last_miss_t < self.miss_every:
            return ""
        self._last_miss_t = t
        self.miss_dir.mkdir(parents=True, exist_ok=True)
        name = f"miss_{wall_ms}.png"
        if not self.writer.save(self.miss_dir / name, rgb):
            return ""                      # kuyruk dolu -- örnek atlandı
        self.misses_saved += 1
        return name

    def close(self) -> None:
        self.writer.close()
        for f in (self._readings, self._frames_csv):
            try:
                f.close()
            except Exception:
                pass

    # --------------------------------------------------------------- kayıtlar
    def _new_record(self, payload, rgb, t, area, poly, barcodes) -> Scan:
        seq = len(self.records) + 1
        name = f"{seq:04d}_{safe_name(payload)}.png"
        saved = self._save_frame(rgb, name)

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
            image_saved=saved,
            barcodes=list(dict.fromkeys(barcodes)),
            truth=None if info is None else {
                "entity": info["entity"], "row": info["row"],
                "bay": info["bay"], "level": info["level"],
                "label_pose_xyzrpy": info["label_pose_xyzrpy"],
            },
        )
        self.records[payload] = rec
        return rec

    def _save_frame(self, rgb: np.ndarray, name: str) -> bool:
        """Kareyi yazıcı kuyruğuna koyar. Kuyruk doluysa False."""
        return self.writer.save(self.frames_dir / name, rgb)

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
        span = (self.last_t - self.first_t) if self.first_t is not None else 0.0
        if span > 0:
            print(f"etkin hız         : {n/span:.1f} Hz "
                  f"({span:.1f} s boyunca)")
        print(f"tahmini düşen kare: {self.est_drops} "
              f"(en büyük boşluk {self.max_gap_s:.2f} s)")
        print(f"kare yazımı       : {self.writer.written} yazıldı"
              + (f", {self.writer.dropped} DÜŞTÜ (kuyruk doldu)"
                 if self.writer.dropped else ""))
        print(f"ham okuma         : {self.n_readings}")
        print(f"taranan kutu      : {len(self.records)}")
        if self.linker is not None:
            pct = 100 * self.bar_linked / self.bar_seen if self.bar_seen else 0
            print(f"barkod            : {len(self.barcode_of)} kutuya bağlandı "
                  f"({self.bar_seen} okuma, %{pct:.0f}'i bir QR'a bağlandı)")
        if self.misses_saved:
            print(f"boş kare örneği   : {self.misses_saved} -> {self.miss_dir}")
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
            # Duvar saati kareyi ALDIĞIMIZ anda okunuyor, çözmeden önce:
            # çözme 70-215 ms sürüyor ve sonda okunsaydı poz eşlemesine o
            # kadar sistematik gecikme girerdi (1.5 m/s'de ~30 cm).
            wall_ms = int(time.time() * 1000)
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
            for payload in scanner.process(rgb, stamp, wall_ms):
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
        # Replay'de duvar saati anlamsız (kareler eski); yine de sütun boş
        # kalmasın diye sanal saat sim zamanıyla aynı ilerletiliyor.
        for payload in scanner.process(rgb, i * dt, int(i * dt * 1000)):
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
                    help="Code128'i de çöz ve kutulara bağla (+0.4 ms/kare)")
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
    ap.add_argument("--camera", default="",
                    help="okuma kayıtlarına yazılacak kamera adı "
                         "(boşsa konu adından türetilir)")
    ap.add_argument("--nominal-hz", type=float, default=20.0,
                    help="kameranın yayın hızı; düşen kare kestirimi bundan")
    ap.add_argument("--miss-every", type=float, default=3.0,
                    help="boş kare örneği saklama aralığı (s), 0 = kapalı")
    ap.add_argument("--miss-max", type=int, default=40,
                    help="en fazla kaç boş kare örneği saklansın")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml",
                    help="barkod ROI geometrisi buradan türetilir")
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
    # "/warehouse_scout/camera_front/image" -> "front"
    parts = args.topic.strip("/").split("/")
    camera = args.camera or (parts[-2].replace("camera_", "")
                             if len(parts) >= 2 else parts[-1])
    linker = None
    if args.with_barcode:
        import yaml
        cfg = yaml.safe_load(args.config.read_text())
        codes = cfg["codes"]
        side, rise = gl.box_label_geometry(codes["box_label"],
                                           codes["texture_px_per_m"],
                                           codes["max_texture_px"])
        linker = BarcodeLinker(cfg, side, rise)
        print(f"barkod açık: kutuya bağlama, QR'ın {linker.drop_m*1000:.0f} mm "
              f"altındaki beklenen konumdan")

    scanner = Scanner(args.out, truth, args.relock, args.with_barcode,
                      camera=camera, nominal_hz=args.nominal_hz,
                      miss_every=args.miss_every if args.miss_every > 0 else 1e9,
                      miss_max=args.miss_max, linker=linker)

    try:
        rc = run_replay(args, scanner) if args.replay else run_ros(args, scanner)
        # Özetten ÖNCE kapat: yazıcı kuyruğu boşalsın ki sayılar kesin olsun.
        scanner.close()
        scanner.summary()
    finally:
        scanner.close()                    # idempotent (hata yolunda da kapat)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
