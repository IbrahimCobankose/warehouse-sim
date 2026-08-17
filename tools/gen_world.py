#!/usr/bin/env python3
"""config/warehouse.yaml'dan Gazebo depo world'ünü ve etiket dokularını üretir.

World elle yazılmak yerine üretiliyor çünkü:
  * her kod benzersiz, onlarca doku ve yerleşim elle sürdürülemez;
  * sentetik veri aşamasında her kodun dünya koordinatındaki pozunu bilmek
    gerekiyor -- üretici bunu ground truth olarak yazıyor;
  * ileride domain randomization için tohumu değiştirip yeniden üretmek yeter.

    .venv/bin/python tools/gen_world.py

Çıktılar:
    gz/models/warehouse_assets/    dokular + etiket quad mesh'i
    gz/worlds/warehouse.sdf        world
    out/ground_truth.json          her kodun yükü, pozu ve boyutu
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gen_labels as gl  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ASSET_MODEL = "warehouse_assets"

# Zemin dokusu karo boyu (m). Optik akış sensörü zemindeki GÖRSEL ÖZELLİKLERİ
# izler; düz renk zeminde iz yok -> "2/20 eşleşme" -> L1'de bozuk hız -> runaway
# (2026-08-13 kök neden). Beton benekli doku tekrarlı UV ile döşenir; karo ~0.5 m
# olunca L1'de (0.4 m irtifa, ~0.7 m alt-kamera FOV) kadrajda bol özellik olur.
FLOOR_TILE_M = 0.5


def floor_texture(px: int = 256, seed: int = 7) -> "Image.Image":
    """Beton benekli, tekrarlanabilir zemin dokusu. Optik akış için asıl olan
    YÜKSEK FREKANSLI kontrast (özellik köşeleri); ince benek + orta ölçek leke."""
    rng = np.random.default_rng(seed)
    fine = rng.normal(0.0, 0.10, (px, px))                    # ince benek
    cs = rng.normal(0.0, 1.0, (px // 16, px // 16))           # orta ölçek
    cs = (cs - cs.min()) / (np.ptp(cs) + 1e-9)
    coarse = np.asarray(Image.fromarray((cs * 255).astype(np.uint8))
                        .resize((px, px), Image.BILINEAR), dtype=np.float64) / 255.0 - 0.5
    g = np.clip(0.5 + fine + 0.15 * coarse, 0.18, 0.82)
    a = (g * 255).astype(np.uint8)
    rgb = np.stack([a, a, np.clip(a.astype(int) + 4, 0, 255).astype(np.uint8)], -1)
    return Image.fromarray(rgb, "RGB")


def floor_mesh_obj(L: float, W: float, tile_m: float) -> str:
    """Zemin quad'ı; UV 0..(L/tile) x 0..(W/tile) -> doku tekrarlı döşenir
    (gz albedo_map varsayılan sarma REPEAT). Gerçek boyutta, SDF scale 1."""
    ru, rv = L / tile_m, W / tile_m
    return f"""# Zemin quad -- gen_world.py üretti; UV {ru:.1f}x{rv:.1f} tekrar (~{tile_m} m/karo)
v {-L/2:.4f} {-W/2:.4f} 0.0
v {L/2:.4f} {-W/2:.4f} 0.0
v {L/2:.4f} {W/2:.4f} 0.0
v {-L/2:.4f} {W/2:.4f} 0.0
vt 0 0
vt {ru:.4f} 0
vt {ru:.4f} {rv:.4f}
vt 0 {rv:.4f}
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

# Etiket quad'ı: XY düzleminde 1x1 m, normali +Z, UV [0,1].
# Kendi mesh'imizi üretiyoruz çünkü SDF <box>/<plane> primitiflerinde UV'nin
# hangi yüze nasıl oturduğu garanti değil; aynalanmış bir doku QR'ı ve barkodu
# okunamaz yapar. 4 köşeli bir OBJ'de belirsizlik kalmıyor.
LABEL_QUAD_OBJ = """# Etiket quad'ı -- gen_world.py tarafından üretildi
# XY düzlemi, normal +Z, u -> +X, v -> +Y (v=1 dokunun üst satırı)
v -0.5 -0.5 0.0
v  0.5 -0.5 0.0
v  0.5  0.5 0.0
v -0.5  0.5 0.0
vt 0.0 0.0
vt 1.0 0.0
vt 1.0 1.0
vt 0.0 1.0
vn 0.0 0.0 1.0
# Dört köşe de aynı normali paylaşır -- yüzlerde normal indeksi hep 1.
# (2/3 yazmak "vertex normal indices out of bounds" verip mesh'i düşürüyor.)
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>warehouse_assets</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Depo world'ünün paylaşılan varlıkları: barkod/QR etiket dokuları ve
    etiket quad mesh'i. Bu model doğrudan spawn edilmez; world SDF'i
    içindeki dosyalara model:// ile başvurur.
  </description>
</model>
"""

# Bu model spawn edilmek için değil, sadece kaynak yolu çözümlemesi için var.
ASSET_STUB_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="warehouse_assets">
    <static>true</static>
    <link name="link"/>
  </model>
</sdf>
"""

# Etiketin gömüldüğü yüzeyden ne kadar önde durduğu. Sıfır olursa z-fighting
# olur, çok büyük olursa etiket havada durur.
LABEL_STANDOFF = 0.004

# QR ile barkod arasındaki dikey boşluk. İkisi tek dikey blok olarak kutunun
# ön yüzü içine ortalanıyor (bkz. inventory() içindeki `margin` hesabı) -- en
# küçük kutuda (S, dz=0.30) bile taşmasın diye.
# MODÜL DÜZEYİNDE: scan_boxes barkodu QR'a göre konumlandırırken bu değere
# ihtiyaç duyuyor; iki yerde ayrı tutmak sessiz bir ROI kayması üretirdi.
LABEL_GAP = 0.010


# --------------------------------------------------------------------------
# SDF yardımcıları
# --------------------------------------------------------------------------

def fmt(v: float) -> str:
    return f"{v:.6g}"


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0) -> str:
    return " ".join(fmt(v) for v in (x, y, z, roll, pitch, yaw))


def box_visual(name: str, size, xyz, rgb, ind="        ") -> str:
    r, g, b = rgb
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
{ind}  <material>
{ind}    <ambient>{fmt(r*0.5)} {fmt(g*0.5)} {fmt(b*0.5)} 1</ambient>
{ind}    <diffuse>{fmt(r)} {fmt(g)} {fmt(b)} 1</diffuse>
{ind}    <specular>0.05 0.05 0.05 1</specular>
{ind}    <pbr><metal>
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>0.9</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


def box_collision(name: str, size, xyz, ind="        ") -> str:
    return f"""{ind}<collision name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
{ind}</collision>
"""


def label_visual(name: str, texture: str, size_wh, xyz_rpy, ind="        ") -> str:
    """Etiket quad'ı. Metalness 0 / roughness 1: kod üzerinde parlama olursa
    okunmaz, o yüzden tamamen mat."""
    w, h = size_wh
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz_rpy)}</pose>
{ind}  <geometry>
{ind}    <mesh>
{ind}      <uri>model://{ASSET_MODEL}/meshes/label_quad.obj</uri>
{ind}      <scale>{fmt(w)} {fmt(h)} 1</scale>
{ind}    </mesh>
{ind}  </geometry>
{ind}  <material>
{ind}    <ambient>1 1 1 1</ambient>
{ind}    <diffuse>1 1 1 1</diffuse>
{ind}    <specular>0 0 0 1</specular>
{ind}    <pbr><metal>
{ind}      <albedo_map>model://{ASSET_MODEL}/materials/textures/{texture}</albedo_map>
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>1.0</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


#: Etiketin dünya yönelimi. Quad'ın normali yerelde +Z; roll=+90 onu -Y'ye
#: çevirir ve dokunun üst kenarı +Z'ye bakar. Yaw=180 ile +Y'ye döner.
FACE_NEG_Y = (math.pi / 2, 0.0, 0.0)
FACE_POS_Y = (math.pi / 2, 0.0, math.pi)


def facing_rpy(facing: int):
    return FACE_POS_Y if facing > 0 else FACE_NEG_Y


# --------------------------------------------------------------------------
# world parçaları
# --------------------------------------------------------------------------

def building(cfg, textures) -> str:
    b = cfg["building"]
    L, W, H, t = b["length"], b["width"], b["height"], b["wall_thickness"]
    parts = [f'  <model name="warehouse_building">\n    <static>true</static>\n    <link name="structure">\n']

    # zemin -- BETON DOKULU (optik akış için; düz renk zemin akışı öldürüyordu,
    # bkz. FLOOR_TILE_M). Görsel = tekrarlı-UV dokulu quad (z=0 yüzeyi, hafif
    # yukarıda ki z-fighting olmasın); collision hâlâ kutu.
    textures["floor.png"] = floor_texture()
    parts.append(f"""      <visual name="floor_v">
        <pose>0 0 0.002 0 0 0</pose>
        <geometry><mesh><uri>model://{ASSET_MODEL}/meshes/floor_tile.obj</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.04 0.04 0.04 1</specular>
          <pbr><metal>
            <albedo_map>model://{ASSET_MODEL}/materials/textures/floor.png</albedo_map>
            <metalness>0.0</metalness>
            <roughness>0.95</roughness>
          </metal></pbr>
        </material>
      </visual>
""")
    parts.append(box_collision("floor_c", (L, W, t), (0, 0, -t / 2), "      "))
    # tavan
    parts.append(box_visual("ceiling_v", (L, W, t), (0, 0, H + t / 2), (0.80, 0.80, 0.82), "      "))
    parts.append(box_collision("ceiling_c", (L, W, t), (0, 0, H + t / 2), "      "))

    walls = [
        ("wall_xp", (t, W + 2 * t, H), (L / 2 + t / 2, 0, H / 2)),
        ("wall_xn", (t, W + 2 * t, H), (-L / 2 - t / 2, 0, H / 2)),
        ("wall_yp", (L + 2 * t, t, H), (0, W / 2 + t / 2, H / 2)),
        ("wall_yn", (L + 2 * t, t, H), (0, -W / 2 - t / 2, H / 2)),
    ]
    for name, size, xyz in walls:
        parts.append(box_visual(f"{name}_v", size, xyz, (0.78, 0.78, 0.75), "      "))
        parts.append(box_collision(f"{name}_c", size, xyz, "      "))

    parts.append("    </link>\n  </model>\n")
    return "".join(parts)


def racking(cfg) -> str:
    rk = cfg["racking"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    ft, uh = rk["frame_thickness"], rk["upright_height"]
    levels, x0 = rk["level_heights"], rk["x_origin"]

    upright_rgb = (0.72, 0.30, 0.08)   # turuncu dikme
    beam_rgb = (0.10, 0.26, 0.55)      # mavi kiriş
    deck_rgb = (0.55, 0.56, 0.58)      # galvaniz raf tablası

    out = []
    for row in rk["rows"]:
        rid, y0 = row["id"], row["y0"]
        out.append(f'  <model name="rack_{rid}">\n    <static>true</static>\n    <link name="frame">\n')
        yc = y0 + depth / 2

        # dikmeler: her göz sınırında, derinliğin ön ve arkasında
        for i in range(nb + 1):
            x = x0 + i * bw
            for tag, y in (("f", y0 + ft / 2), ("b", y0 + depth - ft / 2)):
                out.append(box_visual(f"up_{i}_{tag}_v", (ft, ft, uh), (x, y, uh / 2),
                                      upright_rgb, "      "))
                out.append(box_collision(f"up_{i}_{tag}_c", (ft, ft, uh), (x, y, uh / 2), "      "))

        for li, z in enumerate(levels):
            for bi in range(nb):
                xc = x0 + (bi + 0.5) * bw
                # ön ve arka kirişler
                for tag, y in (("f", y0 + ft / 2), ("b", y0 + depth - ft / 2)):
                    out.append(box_visual(f"beam_{li}_{bi}_{tag}_v", (bw - ft, ft, ft),
                                          (xc, y, z - ft / 2), beam_rgb, "      "))
                    out.append(box_collision(f"beam_{li}_{bi}_{tag}_c", (bw - ft, ft, ft),
                                             (xc, y, z - ft / 2), "      "))
                # raf tablası: kutuların üstünde durduğu yüzey
                out.append(box_visual(f"deck_{li}_{bi}_v", (bw - ft, depth - 2 * ft, 0.02),
                                      (xc, yc, z - 0.01), deck_rgb, "      "))
                out.append(box_collision(f"deck_{li}_{bi}_c", (bw - ft, depth - 2 * ft, 0.02),
                                         (xc, yc, z - 0.01), "      "))
        out.append("    </link>\n  </model>\n")
    return "".join(out)


def inventory(cfg, rng, textures, manifest) -> str:
    """Raflardaki kutular, üzerlerindeki QR etiketleri ve konum barkodları."""
    rk, bx, codes = cfg["racking"], cfg["boxes"], cfg["codes"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    levels, x0 = rk["level_heights"], rk["x_origin"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    spec = codes["box_label"]
    lw, lh = spec["label"]
    pc_spec = codes["box_placard"]
    pw, ph = pc_spec["label"]
    label_gap = LABEL_GAP

    out = ['  <model name="inventory">\n    <static>true</static>\n']
    n_box = 0

    for row in rk["rows"]:
        rid, y0, facing = row["id"], row["y0"], row["facing"]
        # ürün yüzü: koridora bakan kenar
        y_face = (y0 + depth) if facing > 0 else y0

        for bi in range(nb):
            for li, z in enumerate(levels):
                if rng.random() > bx["fill_probability"]:
                    continue
                count = rng.randint(*bx["per_slot"])
                sizes = [rng.choice(bx["sizes"]) for _ in range(count)]
                total_w = sum(s["dims"][0] for s in sizes)
                if total_w > bw - rk["frame_thickness"] - 0.1:
                    sizes = sizes[:1]
                    total_w = sizes[0]["dims"][0]

                # gözün içinde yatayda ortala, aralarına eşit boşluk koy
                gap = (bw - rk["frame_thickness"] - total_w) / (len(sizes) + 1)
                cursor = x0 + bi * bw + rk["frame_thickness"] / 2 + gap

                for si, size in enumerate(sizes):
                    dx, dy, dz = size["dims"]
                    cx = cursor + dx / 2
                    cursor += dx + gap
                    # kutunun ön yüzü raf ön kenarından `front_gap` içeride
                    if facing > 0:
                        cy = y_face - bx["front_gap"] - dy / 2
                        y_label = cy + dy / 2
                    else:
                        cy = y_face + bx["front_gap"] + dy / 2
                        y_label = cy - dy / 2
                    cz = z + dz / 2

                    sku = f"SKU{rng.randint(10000, 99999)}"
                    payload = gl.box_payload(sku, rid, bi + 1, li + 1)
                    tex = f"box_{rid}{bi+1:02d}{li+1}{si}.png"
                    pc_payload = gl.placard_payload(rid, bi + 1, li + 1)
                    pc_caption = gl.placard_caption(rid, bi + 1, li + 1)
                    pc_tex = f"placard_{rid}{bi+1:02d}{li+1}{si}.png"

                    img, module_m = gl.make_box_label(payload, sku, spec, ppm, maxpx)
                    textures[tex] = img
                    pc_img, pc_module_m = gl.make_bay_placard(pc_payload, pc_caption,
                                                              pc_spec, ppm, maxpx)
                    textures[pc_tex] = pc_img

                    link = f"box_{rid}_{bi+1:02d}_{li+1}_{si}"
                    shade = rng.uniform(0.88, 1.06)
                    cardboard = tuple(min(1.0, c * shade) for c in (0.68, 0.52, 0.34))

                    out.append(f'    <link name="{link}">\n')
                    out.append(box_visual("body", (dx, dy, dz), (cx, cy, cz), cardboard, "      "))
                    out.append(box_collision("body_c", (dx, dy, dz), (cx, cy, cz), "      "))

                    # QR + barkod tek dikey blok olarak kutunun ön yüzüne
                    # ortalanır. dz - stack_h negatif çıkarsa (kutu bu iki
                    # etiketi sığdıramayacak kadar alçaksa) margin sıfıra
                    # kenetlenir ve etiketler kutu sınırının biraz dışına taşar
                    # -- config'teki box boyutlarıyla box_placard/box_label
                    # ölçüleri arasında bir tutarsızlık olduğunun işareti.
                    stack_h = lh + label_gap + ph
                    margin = max(0.0, (dz - stack_h) / 2)
                    qr_z = cz + dz / 2 - margin - lh / 2
                    pc_z = qr_z - lh / 2 - label_gap - ph / 2

                    off = LABEL_STANDOFF * (1 if facing > 0 else -1)
                    rpy = facing_rpy(facing)
                    out.append(label_visual("label", tex, (lw, lh),
                                            (cx, y_label + off, qr_z, *rpy), "      "))
                    out.append(label_visual("placard", pc_tex, (pw, ph),
                                            (cx, y_label + off, pc_z, *rpy), "      "))
                    out.append("    </link>\n")

                    manifest.append({
                        "type": "box_qr",
                        "symbology": "QR",
                        "payload": payload,
                        "caption": sku,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [round(cx, 4), round(y_label + off, 4),
                                              round(qr_z, 4), *[round(v, 6) for v in rpy]],
                        "label_size_m": [lw, lh],
                        "module_size_m": round(module_m, 6),
                        "normal": [0.0, float(facing), 0.0],
                    })
                    manifest.append({
                        "type": "box_placard",
                        "symbology": "CODE128",
                        "payload": pc_payload,
                        "caption": pc_caption,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [round(cx, 4), round(y_label + off, 4),
                                              round(pc_z, 4), *[round(v, 6) for v in rpy]],
                        "label_size_m": [pw, ph],
                        "module_size_m": round(pc_module_m, 6),
                        "normal": [0.0, float(facing), 0.0],
                    })
                    n_box += 1

    out.append("  </model>\n")
    print(f"  kutu           : {n_box}")
    return "".join(out)


def floor_markers(cfg, textures, manifest) -> str:
    """Koridor zeminine yapıştırılmış AprilTag markörler -- alt kameranın
    hedefi, GPS'siz uçuşta mutlak konum düzeltmesi için. Tag ID'ler yerleşim
    sırasına göre ardışık atanır (0, 1, 2, ...); talimat kodlanmıyor, sadece
    kimlik -- rota mantığı ayrı bir dosyada tutulacak."""
    rk, codes, b = cfg["racking"], cfg["codes"], cfg["building"]
    spec = codes["floor_marker"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    lw, lh = spec["label"]
    x0 = rk["x_origin"]
    span = rk["bay_count"] * rk["bay_width"]

    out = ['  <model name="floor_markers">\n    <static>true</static>\n']
    n = 0
    for aisle in rk["aisles"]:
        aid, yc = aisle["id"], aisle["y_center"]
        x = x0
        while x <= x0 + span + 1e-6:
            tag_id = n
            caption = f"A{aid} X{x:+.0f}"
            tex = f"marker_a{aid}_{int(round(x))}.png".replace("-", "n")
            img, module_m = gl.make_floor_marker(tag_id, caption, spec, ppm, maxpx)
            textures[tex] = img

            link = f"marker_a{aid}_{int(round(x))}".replace("-", "n")
            out.append(f'    <link name="{link}">\n')
            # zemine yatık: quad normali zaten +Z
            out.append(label_visual("label", tex, (lw, lh),
                                    (x, yc, LABEL_STANDOFF, 0, 0, 0), "      "))
            out.append("    </link>\n")

            manifest.append({
                "type": "floor_marker",
                "symbology": "APRILTAG_36H11",
                "tag_id": tag_id,
                "payload": f"APRILTAG36H11:{tag_id}",
                "caption": caption,
                "entity": f"floor_markers::{link}",
                "aisle": aid,
                "label_pose_xyzrpy": [round(x, 4), round(yc, 4), LABEL_STANDOFF, 0.0, 0.0, 0.0],
                "label_size_m": [lw, lh],
                "module_size_m": round(module_m, 6),
                "normal": [0.0, 0.0, 1.0],
            })
            n += 1
            x += spec["spacing"]

    # Dönüş bölgesi markörleri (rafın ucundaki açık alan). Koridor tag'lerinin
    # HEMEN ardından id alırlar; alt kamera dönüş boyunca bunları görüp EV
    # verir. Rota köşeleri (route.yaml xy waypoint'leri) bunların üstünde.
    n_turn = 0
    for tx, ty in (spec.get("turnarounds") or []):
        tag_id = n
        caption = f"T X{tx:+.0f} Y{ty:+.0f}"
        key = f"{int(round(tx))}_{int(round(ty))}".replace("-", "n")
        tex, link = f"marker_t_{key}.png", f"marker_t_{key}"
        img, module_m = gl.make_floor_marker(tag_id, caption, spec, ppm, maxpx)
        textures[tex] = img
        out.append(f'    <link name="{link}">\n')
        out.append(label_visual("label", tex, (lw, lh),
                                (tx, ty, LABEL_STANDOFF, 0, 0, 0), "      "))
        out.append("    </link>\n")
        manifest.append({
            "type": "floor_marker",
            "symbology": "APRILTAG_36H11",
            "tag_id": tag_id,
            "payload": f"APRILTAG36H11:{tag_id}",
            "caption": caption,
            "entity": f"floor_markers::{link}",
            "aisle": 0,                     # dönüş bölgesi, tek koridora ait değil
            "label_pose_xyzrpy": [round(tx, 4), round(ty, 4), LABEL_STANDOFF,
                                  0.0, 0.0, 0.0],
            "label_size_m": [lw, lh],
            "module_size_m": round(module_m, 6),
            "normal": [0.0, 0.0, 1.0],
        })
        n += 1
        n_turn += 1

    out.append("  </model>\n")
    print(f"  zemin AprilTag : {n}  (koridor {n - n_turn}, dönüş {n_turn})")
    return "".join(out)


def rack_markers(cfg, textures, manifest) -> str:
    """Raf dikmelerine dikey AprilTag'ler -- ÖN kameranın lokalizasyon hedefi.
    Tarama sırasında araç yaw ±90 ile raf yüzüne bakıyor; ön kamera raf yüzünü
    KESİNTİSİZ gördüğü için bu tag'ler floor tag'lerin 4 m'lik boşluğunu (araç
    orada salınıyordu) kapatıyor. Her koridora bakan raf yüzünde, göz sınırı
    dikmelerinde, üç tarama irtifasında bir tag. Floor'la aynı aile ama ID'ler
    floor'un bittiği yerden (10) devam ediyor -- lokalizasyon ikisini id ile
    ayırıyor, floor için alt kamera / raf için ön kamera."""
    rk, codes = cfg["racking"], cfg["codes"]
    spec = codes["rack_marker"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    lw, lh = spec["label"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    x0 = rk["x_origin"]
    heights = spec["heights"]

    # floor tag'ler zaten 0..(n-1); raf tag'leri oradan devam etsin ki
    # tag36h11 uzayında id çakışması olmasın.
    tag_id = sum(1 for c in manifest if c["type"] == "floor_marker")
    out = ['  <model name="rack_markers">\n    <static>true</static>\n']
    n = 0
    for row in rk["rows"]:
        rid, y0, facing = row["id"], row["y0"], row["facing"]
        # koridora bakan ürün yüzü düzlemi (kutu etiketleriyle aynı)
        y_face = (y0 + depth) if facing > 0 else y0
        off = LABEL_STANDOFF * (1 if facing > 0 else -1)
        rpy = facing_rpy(facing)
        for i in range(nb + 1):
            x = x0 + i * bw                      # göz sınırı dikmesinin x'i
            for li, h in enumerate(heights):
                caption = f"{rid}{i}L{li+1}"
                tex = f"rackmark_{rid}_{i}_{li+1}.png"
                img, module_m = gl.make_floor_marker(tag_id, caption, spec, ppm, maxpx)
                textures[tex] = img

                link = f"rackmark_{rid}_{i}_{li+1}"
                out.append(f'    <link name="{link}">\n')
                out.append(label_visual("label", tex, (lw, lh),
                                        (x, y_face + off, h, *rpy), "      "))
                out.append("    </link>\n")

                manifest.append({
                    "type": "rack_marker",
                    "symbology": "APRILTAG_36H11",
                    "tag_id": tag_id,
                    "payload": f"APRILTAG36H11:{tag_id}",
                    "caption": caption,
                    "entity": f"rack_markers::{link}",
                    "row": rid, "upright": i, "level": li + 1,
                    "facing": facing,
                    "label_pose_xyzrpy": [round(x, 4), round(y_face + off, 4),
                                          round(h, 4), *[round(v, 6) for v in rpy]],
                    "label_size_m": [lw, lh],
                    "module_size_m": round(module_m, 6),
                    "normal": [0.0, float(facing), 0.0],
                })
                tag_id += 1
                n += 1
    out.append("  </model>\n")
    print(f"  raf AprilTag   : {n}")
    return "".join(out)


def lighting(cfg) -> str:
    lt = cfg["lighting"]["ceiling_lights"]
    dr, dg, db, da = lt["diffuse"]
    sr, sg, sb, sa = lt["specular"]
    out = []
    i = 0
    for x in lt["x_positions"]:
        for y in lt["y_positions"]:
            # cast_shadows kapalı: nokta ışık gölgeleri OGRE2'de pahalı ve
            # iGPU'da kare hızını yarıya düşürüyor. Kodların okunması için
            # gölge değil, düzgün ve parlamasız aydınlatma gerekiyor.
            out.append(f"""  <light type="point" name="ceiling_{i}">
    <pose>{pose(x, y, lt['height'])}</pose>
    <cast_shadows>false</cast_shadows>
    <diffuse>{fmt(dr)} {fmt(dg)} {fmt(db)} {fmt(da)}</diffuse>
    <specular>{fmt(sr)} {fmt(sg)} {fmt(sb)} {fmt(sa)}</specular>
    <attenuation>
      <range>{fmt(lt['attenuation_range'])}</range>
      <constant>0.3</constant>
      <linear>0.05</linear>
      <quadratic>0.005</quadratic>
    </attenuation>
  </light>
""")
            i += 1
    print(f"  tavan lambası  : {i}")
    return "".join(out)


# --------------------------------------------------------------------------
# birleştirme
# --------------------------------------------------------------------------

def build(cfg) -> tuple[str, list]:
    rng = random.Random(cfg["seed"])
    textures: dict = {}
    manifest: list = []
    lg = cfg["lighting"]
    ar, ag, ab, aa = lg["ambient"]
    br, bg, bb, ba = lg["background"]

    body = [
        building(cfg, textures),
        racking(cfg),
        inventory(cfg, rng, textures, manifest),
        floor_markers(cfg, textures, manifest),
        rack_markers(cfg, textures, manifest),
        lighting(cfg),
    ]

    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- tools/gen_world.py tarafından üretildi -- elle düzenlemeyin.
     Değişiklik için config/warehouse.yaml'ı düzenleyip yeniden çalıştırın. -->
<sdf version="1.9">
  <world name="warehouse">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Sistem eklentileri world'de tanımlanmaz; PX4 bunları
         src/modules/simulation/gz_bridge/server.config ile yükler
         (GZ_SIM_SERVER_CONFIG_PATH). PX4'ün kendi world'leri de aynı
         şekilde çalışıyor. -->

    <scene>
      <grid>false</grid>
      <ambient>{fmt(ar)} {fmt(ag)} {fmt(ab)} {fmt(aa)}</ambient>
      <background>{fmt(br)} {fmt(bg)} {fmt(bb)} {fmt(ba)}</background>
      <shadows>true</shadows>
    </scene>

    <!-- Kapalı ortam: yönlü güneş ışığı yok, aydınlatma tavan lambalarından.
         Yine de PX4'ün NavSat'ı için bir referans konum gerekiyor. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>41.015137</latitude_deg>
      <longitude_deg>28.979530</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

{''.join(body)}  </world>
</sdf>
"""
    return sdf, manifest, textures


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT)
    ap.add_argument("--seed", type=int, help="config'deki tohumu geçersiz kıl")
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    if args.seed is not None:
        cfg["seed"] = args.seed

    print(f"world üretiliyor (tohum {cfg['seed']}):")
    sdf, manifest, textures = build(cfg)

    assets = args.out / "gz" / "models" / ASSET_MODEL
    tex_dir = assets / "materials" / "textures"
    mesh_dir = assets / "meshes"
    for d in (tex_dir, mesh_dir, args.out / "gz" / "worlds", args.out / "out"):
        d.mkdir(parents=True, exist_ok=True)

    # eski dokuları temizle: tohum veya yerleşim değişince artık dosyalar kalmasın
    for old in tex_dir.glob("*.png"):
        old.unlink()

    (assets / "model.config").write_text(MODEL_CONFIG)
    (assets / "model.sdf").write_text(ASSET_STUB_SDF)
    (mesh_dir / "label_quad.obj").write_text(LABEL_QUAD_OBJ)
    (mesh_dir / "floor_tile.obj").write_text(floor_mesh_obj(
        cfg["building"]["length"], cfg["building"]["width"], FLOOR_TILE_M))
    for name, img in textures.items():
        img.save(tex_dir / name, optimize=True)

    world_path = args.out / "gz" / "worlds" / "warehouse.sdf"
    world_path.write_text(sdf)

    gt_path = args.out / "out" / "ground_truth.json"
    gt_path.write_text(json.dumps({
        "world": "warehouse",
        "seed": cfg["seed"],
        "codes": manifest,
    }, indent=2, ensure_ascii=False))

    tex_bytes = sum((tex_dir / n).stat().st_size for n in textures)
    print(f"  toplam kod     : {len(manifest)}")
    print(f"  doku           : {len(textures)} dosya, {tex_bytes/1e6:.1f} MB")
    print(f"\n  {world_path.relative_to(PROJECT_ROOT)}")
    print(f"  {gt_path.relative_to(PROJECT_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
