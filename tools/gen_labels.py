#!/usr/bin/env python3
"""Depo simülasyonu için barkod / QR etiket dokuları üretir.

Kodlar piksel piksel elle çizilir (kütüphanelerin kendi render'ı yerine
matrisleri kullanılır). Sebebi: modül başına düşen piksel sayısı bu işin
tamamının belirleyici parametresi -- yeniden boyutlandırma veya kütüphaneye
özgü bir yuvarlama, kodu sessizce okunamaz hale getirebilir. Burada her
modülün kaç piksel olduğu tam olarak bilinir.

Doğrudan çalıştırıldığında okunabilirlik bütçesini raporlar ve örnek
etiketler üretir:

    .venv/bin/python tools/gen_labels.py --budget
    .venv/bin/python tools/gen_labels.py --samples out/
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import qrcode
from barcode import Code128
from barcode.writer import BaseWriter
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent

_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
]


def _font(size_px: int) -> ImageFont.FreeTypeFont:
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            return ImageFont.truetype(path, size_px)
    return ImageFont.load_default()


def _centered_text(draw: ImageDraw.ImageDraw, box, text: str, fill=(0, 0, 0)) -> None:
    """`box` = (x0, y0, x1, y1) dikdörtgenine sığacak en büyük yazıyı ortalar."""
    x0, y0, x1, y1 = box
    max_w, max_h = x1 - x0, y1 - y0
    if max_w <= 2 or max_h <= 2:
        return
    size = max_h
    while size > 4:
        font = _font(size)
        left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
        if right - left <= max_w and bottom - top <= max_h:
            draw.text(
                (x0 + (max_w - (right - left)) / 2 - left,
                 y0 + (max_h - (bottom - top)) / 2 - top),
                text, font=font, fill=fill,
            )
            return
        size -= 1


# --------------------------------------------------------------------------
# okunabilirlik bütçesi
# --------------------------------------------------------------------------

def px_per_m(width_px: int, hfov_rad: float, distance_m: float) -> float:
    """Verilen mesafede görüntünün metre başına kaç piksel çözdüğü."""
    view_width_m = 2.0 * distance_m * math.tan(hfov_rad / 2.0)
    return width_px / view_width_m


def px_per_module(width_px: int, hfov_rad: float, distance_m: float,
                  module_size_m: float) -> float:
    return px_per_m(width_px, hfov_rad, distance_m) * module_size_m


#: Bir kod, modül başına bu kadar pikselin altında güvenilir çözülmez.
MIN_PX_PER_MODULE = 3.0
COMFORT_PX_PER_MODULE = 4.0


@dataclass
class BudgetRow:
    code: str
    camera: str
    distance_m: float
    module_mm: float
    px_per_module: float

    @property
    def verdict(self) -> str:
        if self.px_per_module >= COMFORT_PX_PER_MODULE:
            return "RAHAT"
        if self.px_per_module >= MIN_PX_PER_MODULE:
            return "SINIRDA"
        return "OKUNMAZ"


# --------------------------------------------------------------------------
# QR
# --------------------------------------------------------------------------

def qr_matrix(payload: str, version: int, error_correction: str = "M"):
    """QR modül matrisini döndürür (quiet zone yok). Versiyon sabittir:
    payload sığmazsa hata verir -- sessizce büyümesi modül boyutunu
    küçültüp okunabilirliği bozardı."""
    ec = {
        "L": qrcode.constants.ERROR_CORRECT_L,
        "M": qrcode.constants.ERROR_CORRECT_M,
        "Q": qrcode.constants.ERROR_CORRECT_Q,
        "H": qrcode.constants.ERROR_CORRECT_H,
    }[error_correction]
    qr = qrcode.QRCode(version=version, error_correction=ec, border=0)
    qr.add_data(payload)
    qr.make(fit=False)          # fit=False -> sığmazsa DataOverflowError
    matrix = qr.get_matrix()
    expected = 17 + 4 * version
    assert len(matrix) == expected, f"beklenen {expected} modül, gelen {len(matrix)}"
    return matrix


def qr_module_count(version: int) -> int:
    return 17 + 4 * version


def draw_matrix(img: Image.Image, matrix, origin_px, module_px: int) -> None:
    draw = ImageDraw.Draw(img)
    ox, oy = origin_px
    for r, row in enumerate(matrix):
        for c, on in enumerate(row):
            if on:
                x = ox + c * module_px
                y = oy + r * module_px
                draw.rectangle([x, y, x + module_px - 1, y + module_px - 1], fill=(0, 0, 0))


# --------------------------------------------------------------------------
# AprilTag (zemin markörleri)
# --------------------------------------------------------------------------

#: tag36h11: OpenCV'nin resmi AprilTag sözlüğü. 6x6 veri biti + ailenin
#: sabit standardı olan 1 modüllük siyah çerçeve = 8x8 toplam modül.
#: Bizim tasarım kararımız değil, AprilTag ailesinin tanımı.
_APRILTAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
APRILTAG_MODULES = 8


def apriltag_matrix(tag_id: int):
    """tag36h11 deseni, modül ızgarası olarak (True = siyah).
    generateImageMarker'a modül sayısı kadar piksel istenince kütüphane
    tam 1 piksel/modül, antialiasing'siz saf 0/255 matris üretiyor --
    doğrulandı, ekstra bir eşikleme adımına gerek yok."""
    img = cv2.aruco.generateImageMarker(_APRILTAG_DICT, tag_id, APRILTAG_MODULES)
    return [[bool(v < 128) for v in row] for row in img]


# --------------------------------------------------------------------------
# Code128
# --------------------------------------------------------------------------

class _RawWriter(BaseWriter):
    """python-barcode'un çizim yapmayan writer'ı: sadece modül dizisini almak
    için. Kütüphanenin kendi ImageWriter'ı mm/dpi üzerinden hesaplayıp
    yuvarlıyor; burada modülleri doğrudan çiziyoruz."""

    def __init__(self):
        super().__init__(self._noop, self._noop, self._noop, self._noop)

    @staticmethod
    def _noop(*args, **kwargs):
        return None


def code128_modules(payload: str) -> str:
    """Code128 sembolünün '1'/'0' dizisi (quiet zone hariç)."""
    code = Code128(payload, writer=_RawWriter())
    return "".join(code.build())


# --------------------------------------------------------------------------
# yük (payload) biçimleri -- gen_world.py da bunları kullanır
# --------------------------------------------------------------------------

def placard_payload(row_id: str, bay: int, level: int) -> str:
    """Konum plakası yükü: sıra harfi + göz (2 hane) + seviye (2 hane).
    Rakamları çift sayıda tutmak Code128'in C modunu tetikler ve sembolü
    kısaltır; "A-03-2" gibi bir yazım %28 daha küçük modül verirdi."""
    return f"{row_id}{bay:02d}{level:02d}"


def placard_caption(row_id: str, bay: int, level: int) -> str:
    return f"{row_id}-{bay:02d}-{level}"


def box_payload(sku: str, row_id: str, bay: int, level: int) -> str:
    return f"WH1|{row_id}|{bay:02d}|{level}|{sku}"


def marker_payload(aisle_id: int, x_m: float) -> str:
    return f"WH1|A{aisle_id}|{x_m:+.0f}"


PLACARD_SAMPLE = placard_payload("A", 3, 2)


# --------------------------------------------------------------------------
# etiket üreticileri
# --------------------------------------------------------------------------

def _canvas(label_wh_m, px_per_m_tex: float, max_px: int):
    """Etiket tuvalini oluşturur; uzun kenar `max_px`i aşarsa ölçek düşürülür."""
    w_m, h_m = label_wh_m
    scale = px_per_m_tex
    longest = max(w_m, h_m) * scale
    if longest > max_px:
        scale = max_px / max(w_m, h_m)
    w_px = max(8, int(round(w_m * scale)))
    h_px = max(8, int(round(h_m * scale)))
    return Image.new("RGB", (w_px, h_px), (255, 255, 255)), scale


def make_box_label(payload: str, caption: str, spec: dict,
                   px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Kutu üstündeki QR etiketi. (görüntü, modül_boyutu_m) döndürür."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    n = qr_module_count(spec["qr_version"])
    # Modül pikselini tam sayıya yuvarla: kesirli modül genişliği komşu
    # modüllerin farklı boyutta çıkmasına, yani kodun bozulmasına yol açar.
    module_px = max(1, int(round(spec["code"] * scale / n)))
    qr_px = module_px * n
    module_m = spec["code"] / n

    matrix = qr_matrix(payload, spec["qr_version"], spec.get("qr_error_correction", "M"))
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    qr_area_h = h_px - caption_px
    draw_matrix(img, matrix,
                ((w_px - qr_px) // 2, (qr_area_h - qr_px) // 2), module_px)

    draw = ImageDraw.Draw(img)
    if caption_px > 4:
        pad = max(2, int(0.10 * caption_px))
        _centered_text(draw, (pad, qr_area_h, w_px - pad, h_px - pad), caption)
    # ince çerçeve: etiketi kutunun kartonundan ayırır
    draw.rectangle([0, 0, w_px - 1, h_px - 1], outline=(40, 40, 40), width=max(1, w_px // 220))
    return img, module_m


def make_bay_placard(payload: str, caption: str, spec: dict,
                     px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Konum barkodu (Code128) -- kutunun ön yüzünde, QR etiketin altında."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    bits = code128_modules(payload)
    n = len(bits)
    module_px = max(1, int(round(spec["bar_width"] * scale / n)))
    bars_px = module_px * n
    module_m = spec["bar_width"] / n

    bar_h = int(round(spec["bar_height"] * scale))
    x0 = (w_px - bars_px) // 2
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    y0 = max(0, (h_px - caption_px - bar_h) // 2)

    draw = ImageDraw.Draw(img)
    for i, bit in enumerate(bits):
        if bit == "1":
            x = x0 + i * module_px
            draw.rectangle([x, y0, x + module_px - 1, y0 + bar_h - 1], fill=(0, 0, 0))

    if caption_px > 4:
        pad = max(2, int(0.12 * caption_px))
        _centered_text(draw, (x0, y0 + bar_h + pad, x0 + bars_px, h_px - pad), caption)

    # ÇERÇEVE YOK -- BİLEREK. QR etiketinde olduğu gibi ince bir kenarlık
    # çizmek Code128'i tamamen çözülemez hale getiriyordu: kenarlık, zbar
    # için sessiz alanın (quiet zone) içinde duran bir çubuk oluyor. Etikette
    # çubukların iki yanında (0.320-0.280)/2 = 20 mm boşluk var; bu zaten
    # standardın istediği 10 modülün (35.4 mm) altında, kenarlık da onu
    # 3 modüle indiriyordu. QR'da sorun çıkmıyor çünkü QR'ın sessiz alan
    # şartı 4 modül ve etikette 5 modül var.
    return img, module_m


def make_floor_marker(tag_id: int, caption: str, spec: dict,
                      px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Zemin markörü: AprilTag (tag36h11) + kalın çerçeve (alt kameradan
    bakıldığında zeminden ayrışması için). `caption` sadece görsel hata
    ayıklama içindir, tag'in kendisine kodlanmıyor -- AprilTag yalnızca
    `tag_id`yi taşır."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    n = APRILTAG_MODULES
    module_px = max(1, int(round(spec["code"] * scale / n)))
    tag_px = module_px * n
    module_m = spec["code"] / n

    draw = ImageDraw.Draw(img)
    border = max(2, int(0.02 * min(w_px, h_px)))
    draw.rectangle([0, 0, w_px - 1, h_px - 1], outline=(0, 0, 0), width=border)

    matrix = apriltag_matrix(tag_id)
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    tag_area_h = h_px - caption_px
    draw_matrix(img, matrix, ((w_px - tag_px) // 2, (tag_area_h - tag_px) // 2), module_px)

    if caption_px > 4:
        pad = max(2, int(0.12 * caption_px))
        _centered_text(draw, (border + pad, tag_area_h, w_px - border - pad, h_px - border - pad),
                       caption)
    return img, module_m


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def compute_budget(cfg: dict) -> list[BudgetRow]:
    codes = cfg["codes"]
    cams = {c["name"]: c for c in cfg["cameras"]}
    aisle_half = 1.5  # koridor merkezinden raf yüzüne

    rows: list[BudgetRow] = []
    front = cams["front"]
    box_module = codes["box_label"]["code"] / qr_module_count(codes["box_label"]["qr_version"])
    placard_module = codes["box_placard"]["bar_width"] / len(code128_modules(PLACARD_SAMPLE))
    for d in (1.0, aisle_half, 2.0, 2.5):
        rows.append(BudgetRow("kutu QR", "front", d, box_module * 1000,
                              px_per_module(front["width"], front["hfov"], d, box_module)))
    for d in (1.0, aisle_half, 2.0, 2.5):
        rows.append(BudgetRow("kutu barkodu", "front", d, placard_module * 1000,
                              px_per_module(front["width"], front["hfov"], d, placard_module)))

    down = cams["down"]
    fm = codes["floor_marker"]
    fm_module = fm["code"] / APRILTAG_MODULES
    for h in (1.5, 2.0, 3.0, 4.0):
        rows.append(BudgetRow("zemin AprilTag", "down", h, fm_module * 1000,
                              px_per_module(down["width"], down["hfov"], h, fm_module)))
    return rows


def _load_cfg(path: Path) -> dict:
    import yaml
    with open(path) as fh:
        return yaml.safe_load(fh)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--budget", action="store_true",
                    help="okunabilirlik bütçesini tablo olarak yazdır")
    ap.add_argument("--samples", type=Path, metavar="DIZIN",
                    help="her kod tipinden bir örnek etiket üret")
    args = ap.parse_args()

    cfg = _load_cfg(args.config)

    if args.budget or not args.samples:
        rows = compute_budget(cfg)
        print(f"{'kod':<16}{'kamera':<8}{'mesafe':>8}{'modül':>10}{'px/modül':>11}  sonuç")
        print("-" * 62)
        last = None
        for r in rows:
            if last is not None and r.code != last:
                print()
            last = r.code
            print(f"{r.code:<16}{r.camera:<8}{r.distance_m:>7.1f}m"
                  f"{r.module_mm:>9.2f}mm{r.px_per_module:>11.2f}  {r.verdict}")
        print(f"\neşik: >={MIN_PX_PER_MODULE} px/modül okunur, "
              f">={COMFORT_PX_PER_MODULE} rahat")

    if args.samples:
        out = args.samples
        out.mkdir(parents=True, exist_ok=True)
        codes = cfg["codes"]
        ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
        img, m = make_box_label(box_payload("SKU48213", "A", 3, 2), "SKU48213",
                                codes["box_label"], ppm, maxpx)
        img.save(out / "ornek_kutu_etiketi.png")
        print(f"kutu etiketi     {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm")
        img, m = make_bay_placard(PLACARD_SAMPLE, placard_caption("A", 3, 2),
                                  codes["box_placard"], ppm, maxpx)
        img.save(out / "ornek_kutu_barkodu.png")
        print(f"kutu barkodu     {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm")
        img, m = make_floor_marker(7, "A1 X-8", codes["floor_marker"], ppm, maxpx)
        img.save(out / "ornek_zemin_apriltag.png")
        print(f"zemin AprilTag   {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm")
        print(f"\n-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
