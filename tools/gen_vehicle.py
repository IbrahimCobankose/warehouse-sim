#!/usr/bin/env python3
"""3 kameralı depo drone'unun Gazebo modelini üretir.

Araç PX4'ün x500'ünü olduğu gibi devralır (gövde, 4 rotor, motor eklentileri,
IMU, manyetometre, barometre) ve üzerine ön / alt / arka kameraları ekler.
x500'ü kopyalayıp değiştirmek yerine `<include merge>` ile devralmanın sebebi:
PX4'ün airframe parametreleri (motor sabitleri, atalet, karıştırıcı) x500'e
göre ayarlı; gövdeyi çatallarsak PX4 güncellendiğinde sessizce ayrışır.

    .venv/bin/python tools/gen_vehicle.py

Çıktı:
    gz/models/warehouse_scout/      model.sdf + model.config
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gen_labels as gl  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent

#: DİKKAT -- model adı seçerken: PX4 make hedeflerini `gz_<model>_<world>`
#: biçiminde üretir ve varsayılan world için `gz_<model>`. Bu yüzden
#: "x500_warehouse" adı, "x500" modeli + "warehouse" world'ü ile aynı hedefi
#: üretip CMake'i "target already exists" hatasıyla düşürüyordu. Yeni ad hiçbir
#: <mevcut model>_<mevcut world> kombinasyonuna eşit olmamalı.
MODEL_NAME = "warehouse_scout"

#: Kamera gövdesinin kütlesi. 3 kamera toplam 0.09 kg, 2.0 kg'lık x500'ün
#: %4.5'i. PX4'ün hover thrust kestiricisi bu farkı uçuşta kendisi kapatıyor.
CAMERA_MASS = 0.030
CAMERA_INERTIA = 2.5e-05

MODEL_CONFIG = f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Depo envanter drone'u: x500 tabanlı quadrotor, ön / alt / arka olmak
    üzere 3 kamera. tools/gen_vehicle.py tarafından üretilir.
  </description>
</model>
"""


def camera_link(cam: dict) -> str:
    """Bir kamera link'i + onu gövdeye bağlayan sabit eklem."""
    name = cam["name"]
    link = f"camera_{name}_link"
    x, y, z, roll, pitch, yaw = cam["pose"]
    p = " ".join(f"{v:.6g}" for v in cam["pose"])

    # Konu adı açıkça veriliyor. Varsayılan gz adlandırması world ve model
    # örneği adını içeriyor (/world/warehouse/model/warehouse_scout_0/...),
    # bu da ROS köprüsünü ve kayıt betiklerini kırılgan yapıyor.
    topic = f"{MODEL_NAME}/camera_{name}/image"

    return f"""
    <!-- ================= {name} kamera ================= -->
    <link name="{link}">
      <pose relative_to="base_link">{p}</pose>
      <inertial>
        <mass>{CAMERA_MASS}</mass>
        <inertia>
          <ixx>{CAMERA_INERTIA}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{CAMERA_INERTIA}</iyy><iyz>0</iyz>
          <izz>{CAMERA_INERTIA}</izz>
        </inertia>
      </inertial>
      <visual name="{link}_housing">
        <geometry><box><size>0.025 0.035 0.030</size></box></geometry>
        <material>
          <ambient>0.05 0.05 0.06 1</ambient>
          <diffuse>0.12 0.12 0.14 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      <visual name="{link}_lens">
        <pose>0.014 0 0 0 1.5707963 0</pose>
        <geometry><cylinder><radius>0.009</radius><length>0.008</length></cylinder></geometry>
        <material>
          <ambient>0.25 0.25 0.32 1</ambient>
          <diffuse>0.30 0.30 0.38 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
        </material>
      </visual>
      <sensor name="camera_{name}" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <topic>{topic}</topic>
        <camera>
          <horizontal_fov>{cam['hfov']:.6g}</horizontal_fov>
          <image>
            <width>{cam['width']}</width>
            <height>{cam['height']}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>{cam['near']:.6g}</near>
            <far>{cam['far']:.6g}</far>
          </clip>
          <!-- Gürültü kasıtlı olarak kapalı: barkod okunabilirliğinin
               taban çizgisi burada kurulur. Sağlamlık testi için sentetik
               veri aşamasında gürültü/bulanıklık eklenecek. -->
        </camera>
        <always_on>1</always_on>
        <update_rate>{cam['update_rate']}</update_rate>
        <visualize>false</visualize>
      </sensor>
      <gravity>true</gravity>
      <velocity_decay/>
    </link>
    <joint name="camera_{name}_joint" type="fixed">
      <parent>base_link</parent>
      <child>{link}</child>
    </joint>
"""


def nav_sensors(cfg: dict) -> str:
    """Optik akış + aşağı bakan mesafe ölçer (GPS'siz uçuşun temeli).

    ADLAR SABİT: PX4'ün gz köprüsü `flow_link/sensor/optical_flow` ve
    `lidar_sensor_link/sensor/lidar` yollarına abone oluyor (GZBridge.cpp).
    Farklı adlandırma = sessizce veri gelmemesi.

    Akış sensörünün kendisi PX4'ün `optical_flow` modelinden `include merge`
    ile geliyor: içindeki `gz:type="optical_flow"` özel sensörü, PX4'ün
    server.config'inde yüklenen OpticalFlowSystem eklentisiyle çalışıyor.
    Elle yeniden yazmak yerine devralmak, x500'de olduğu gibi, PX4
    güncellendiğinde ayrışmayı önlüyor.
    """
    s = cfg.get("sensors")
    if not s:
        return ""
    of, rf = s["optical_flow"], s["rangefinder"]
    ofp = " ".join(f"{v:.6g}" for v in of["pose"])
    rfp = " ".join(f"{v:.6g}" for v in rf["pose"])

    return f"""
    <!-- ================= optik akış ================= -->
    <include merge="true">
      <uri>model://optical_flow</uri>
      <pose relative_to="base_link">{ofp} 0 0 0</pose>
    </include>
    <joint name="optical_flow_joint" type="fixed">
      <parent>base_link</parent>
      <child>flow_link</child>
    </joint>

    <!-- ============= aşağı bakan mesafe ölçer =============
         Tek ışın (samples=1). pitch=1.57 + sensörün kendi roll=3.14'ü
         birlikte ışını aşağı çeviriyor; PX4 köprüsü sensörün DÜNYA
         yönelimine bakıp "aşağı bakan" kararını buradan veriyor, o yüzden
         bu iki dönüş x500_flow referansıyla birebir aynı tutuldu. -->
    <link name="lidar_sensor_link">
      <pose relative_to="base_link">{rfp} 0 1.5707963 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-05</iyy><iyz>0</iyz>
          <izz>1e-05</izz>
        </inertia>
      </inertial>
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0 3.14 0 0</pose>
        <update_rate>{rf['update_rate']}</update_rate>
        <ray>
          <scan>
            <horizontal><samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range>
            <min>{rf['min_range']:.6g}</min>
            <max>{rf['max_range']:.6g}</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>1</always_on>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>
"""


def build(cfg: dict) -> str:
    cams = "".join(camera_link(c) for c in cfg["cameras"])
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- tools/gen_vehicle.py tarafından üretildi -- elle düzenlemeyin.
     Kamera ayarları için config/warehouse.yaml -> cameras,
     navigasyon sensörleri için -> sensors. -->
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <!-- Gövde, rotorlar, motor eklentileri ve IMU/mag/baro x500'den gelir. -->
    <include merge="true">
      <uri>model://x500</uri>
    </include>
{cams}{nav_sensors(cfg)}  </model>
</sdf>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT)
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    model_dir = args.out / "gz" / "models" / MODEL_NAME
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.config").write_text(MODEL_CONFIG)
    (model_dir / "model.sdf").write_text(build(cfg))

    print(f"{MODEL_NAME} üretildi:")
    total_px = 0
    for c in cfg["cameras"]:
        px = c["width"] * c["height"] * c["update_rate"]
        total_px += px
        fov_deg = c["hfov"] * 180 / 3.14159265
        print(f"  {c['name']:<6} {c['width']}x{c['height']} @ {c['update_rate']} Hz"
              f"  FOV {fov_deg:.1f}°  poz {c['pose'][:3]}")
    print(f"  toplam render yükü: {total_px/1e6:.1f} Mpiksel/s")
    if cfg.get("sensors"):
        of, rf = cfg["sensors"]["optical_flow"], cfg["sensors"]["rangefinder"]
        print(f"  akış   flow_link @ {of['update_rate']} Hz  poz {of['pose']}")
        print(f"  mesafe lidar_sensor_link @ {rf['update_rate']} Hz  "
              f"poz {rf['pose']}  menzil {rf['min_range']}-{rf['max_range']} m")
    print(f"\n  {(model_dir / 'model.sdf').relative_to(PROJECT_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
