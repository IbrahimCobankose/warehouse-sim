#!/usr/bin/env python3
"""Etiket dokularının gerçekten çözülebildiğini ampirik olarak doğrular.

gen_labels.py'deki px/modül bütçesi analitiktir; kağıt üstünde tutması
kodun gerçekten okunacağı anlamına gelmez. Bu araç etiketi, verilen mesafede
kamera görüntüsünde kaplayacağı piksel boyutuna indirger, hafif bir optik
yumuşama ve sensör gürültüsü ekler, sonra pyzbar ile çözmeyi dener.

Bu bir simülasyon render'ı değil, onun alt sınırını modelleyen bir yaklaşım:
gerçek render'da mipmap ve perspektif ek kayıp getirir, o yüzden buradaki
"okundu" sonucu gerekli şarttır, yeterli değil. Asıl doğrulama world
ayağa kalktıktan sonra kameradan alınan kareyle yapılır.

    .venv/bin/python tools/check_readability.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter
from pyzbar import pyzbar

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gen_labels as gl  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def simulate_capture(label: Image.Image, label_width_m: float,
                     cam: dict, distance_m: float,
                     blur_px: float = 0.6, noise_sigma: float = 2.0,
                     rng: np.random.Generator | None = None) -> Image.Image:
    """Etiketi, `distance_m` mesafeden çekilmiş gibi küçültür.

    Etiketin karşıya tam dik durduğu (en iyi durum) varsayılır.
    """
    rng = rng or np.random.default_rng(0)
    scale = gl.px_per_m(cam["width"], cam["hfov"], distance_m)
    w = max(1, int(round(label_width_m * scale)))
    h = max(1, int(round(w * label.size[1] / label.size[0])))

    # Optik yumuşama küçültmeden ÖNCE uygulanır: gerçek sistemde de bulanıklık
    # sensöre düşmeden önce, tam çözünürlükteki sahnede oluşur.
    src = label.filter(ImageFilter.GaussianBlur(blur_px * label.size[0] / max(w, 1) * 0.05))
    small = src.resize((w, h), Image.LANCZOS)

    arr = np.asarray(small, dtype=np.float32)
    arr += rng.normal(0.0, noise_sigma, arr.shape)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def decode(img: Image.Image) -> list[str]:
    return [d.data.decode("utf-8", "replace") for d in pyzbar.decode(img)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--trials", type=int, default=5,
                    help="mesafe başına farklı gürültü tohumuyla deneme sayısı")
    ap.add_argument("--dump", type=Path, help="küçültülmüş görüntüleri bu dizine yaz")
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    codes, cams = cfg["codes"], {c["name"]: c for c in cfg["cameras"]}
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]

    box_data = gl.box_payload("SKU48213", "A", 3, 2)
    placard_data = gl.placard_payload("A", 3, 2)
    marker_data = gl.marker_payload(1, -8)

    box_img, _ = gl.make_box_label(box_data, "SKU48213", codes["box_label"], ppm, maxpx)
    placard_img, _ = gl.make_bay_placard(placard_data, gl.placard_caption("A", 3, 2),
                                         codes["bay_placard"], ppm, maxpx)
    marker_img, _ = gl.make_floor_marker(marker_data, "A1 X-8", codes["floor_marker"],
                                         ppm, maxpx)

    cases = [
        ("kutu QR", box_img, codes["box_label"]["label"][0], box_data,
         cams["front"], [1.0, 1.25, 1.5, 1.75, 2.0, 2.5]),
        ("raf plakası", placard_img, codes["bay_placard"]["label"][0], placard_data,
         cams["front"], [1.0, 1.25, 1.5, 1.75, 2.0, 2.5]),
        ("zemin markörü", marker_img, codes["floor_marker"]["label"][0], marker_data,
         cams["down"], [1.5, 2.0, 2.5, 3.0, 3.5, 4.0]),
    ]

    if args.dump:
        args.dump.mkdir(parents=True, exist_ok=True)

    failures = 0
    print(f"{'kod':<16}{'mesafe':>8}{'görüntü':>11}{'çözüldü':>12}   içerik")
    print("-" * 72)
    for name, img, width_m, expected, cam, distances in cases:
        for d in distances:
            ok = 0
            sample = None
            for t in range(args.trials):
                cap = simulate_capture(img, width_m, cam, d,
                                       rng=np.random.default_rng(1000 * t + int(d * 100)))
                sample = sample or cap
                results = decode(cap)
                if expected in results:
                    ok += 1
            px = f"{sample.size[0]}x{sample.size[1]}"
            rate = f"{ok}/{args.trials}"
            flag = "" if ok == args.trials else ("  <-- kısmi" if ok else "  <-- BAŞARISIZ")
            print(f"{name:<16}{d:>7.2f}m{px:>11}{rate:>12}   {expected}{flag}")
            if args.dump:
                sample.save(args.dump / f"{name.replace(' ', '_')}_{d:.2f}m.png")
            if ok == 0 and d <= 2.0:
                failures += 1
        print()

    if failures:
        print(f"UYARI: {failures} durum çalışma mesafesi içinde hiç çözülemedi.")
        return 1
    print("Tüm kodlar hedeflenen çalışma mesafelerinde çözüldü.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
