#!/usr/bin/env python3
"""Zemin AprilTag'lerinden mutlak konum kestirir ve PX4'e dış görüş
(visual odometry) olarak besler.

Yol haritasının 3. aşaması. 2. aşama (optik akış) sürekli ama KAYAN bir
tahmin veriyordu; buradaki düzeltme kaymayı, bir markörün üstünden her
geçişte sıfırlıyor.

    source /opt/ros/jazzy/setup.bash
    .venv/bin/python scripts/apriltag_localize.py

Önce hesabın doğruluğunu görmek için (PX4'e hiçbir şey göndermez, gerçek
pozla karşılaştırır):

    .venv/bin/python scripts/apriltag_localize.py --dry-run --compare

NASIL ÇALIŞIR
  Alt kamerada tag36h11 aranır (cv2.aruco). Her tag'in dünya üzerindeki yeri
  BİLİNİYOR -- bu bir altyapı ölçümü, aracın pozu değil: gerçek bir depoda da
  markörler ölçülerek yerleştirilir. Poz her tag'in KENDİ çerçevesinde
  (z=0 düzlemi, merkez orijinde) solvePnP ile çözülür, sonra tag'in bilinen
  dünya konumuyla toplanır. Ara çerçeveyi atlayıp dünya koordinatlarını
  doğrudan solvePnP'ye vermek işe YARAMIYOR -- bkz. tag_object_points().

  Ground truth (Gazebo'nun gerçek pozu) yalnızca --compare ile, yalnızca
  "ne kadar isabetli" sorusunu cevaplamak için okunur; kestirime asla girmez.

NEDEN MAVLink, uXRCE-DDS DEĞİL
  README'nin ilk planı uXRCE-DDS diyordu ama bu makinede ne Micro XRCE-DDS
  Agent ne px4_msgs kurulu. MAVLink VISION_POSITION_ESTIMATE mesajı PX4'te
  AYNI uORB konusuna (vehicle_visual_odometry) düşüyor ve EKF2 tarafından
  aynı şekilde tüketiliyor; projede zaten pymavlink kullanılıyor. İşlevsel
  fark yok, kurulum yükü çok daha az.

PORT: PX4'ün onboard MAVLink'i 14540'ta. test_flight.py / measure_drift.py da
oradan bağlanıyor ve aynı UDP portuna iki süreç bağlanamaz -- bu düğüm bu
yüzden varsayılan olarak 14550'yi (GCS bağlantısı) kullanır.
"""

from __future__ import annotations

import argparse
import json
import math
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
from ros_image import to_array, to_gray                     # noqa: E402

import gen_labels as gl                                     # noqa: E402
from pymavlink import mavutil                               # noqa: E402

DEFAULT_TOPIC = "/warehouse_scout/camera_down/image"
FRONT_TOPIC = "/warehouse_scout/camera_front/image"

sys.stdout.reconfigure(line_buffering=True)


def intrinsics(cam: dict) -> tuple[np.ndarray, np.ndarray]:
    """Gazebo kamerasının K matrisi. Bozulma (distortion) yok."""
    w, h = cam["width"], cam["height"]
    fx = (w / 2.0) / math.tan(cam["hfov"] / 2.0)
    K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1]], dtype=np.float64)
    return K, np.zeros(5)


def load_tag_map(gt_path: Path, cfg: dict) -> tuple[dict[int, np.ndarray], float]:
    """`(tag id -> tag MERKEZİNİN dünya koordinatı, tag_kenarı)`.

    Tag etiketin tam ortasında değil (altta caption şeridi var) ve gerçek
    kenarı modül yuvarlaması yüzünden config'deki 0.34 değil; ikisi de
    gen_labels.floor_marker_geometry'den geliyor -- aynı hesabı burada
    tekrarlamak sessiz bir konum hatası kaynağı olurdu.

    Markörlerin dünya üzerindeki yeri BİLİNEN ALTYAPIDIR, aracın pozu değil:
    gerçek bir depoda da markörler ölçülerek yerleştirilir.
    """
    codes = cfg["codes"]
    size, dx, dy = gl.floor_marker_geometry(
        codes["floor_marker"], codes["texture_px_per_m"], codes["max_texture_px"])

    tags: dict[int, np.ndarray] = {}
    for c in json.loads(gt_path.read_text())["codes"]:
        if c["type"] != "floor_marker":
            continue
        tag_id = int(c["payload"].rsplit(":", 1)[1])
        cx, cy, cz = c["label_pose_xyzrpy"][:3]
        tags[tag_id] = np.array([cx + dx, cy + dy, cz], dtype=np.float64)
    return tags, size


def tag_object_points(size: float) -> np.ndarray:
    """Tag'in KENDİ çerçevesindeki köşeleri: z=0 düzlemi, merkez orijinde,
    aruco sırası (SolÜst, SağÜst, SağAlt, SolAlt).

    DİKKAT -- burası bir kez yanlış yapıldı: solvePnP'ye köşelerin doğrudan
    DÜNYA koordinatlarını vermek cazip ama `SOLVEPNP_IPPE(_SQUARE)` nesne
    noktalarının z=0 düzleminde olmasını ŞART koşuyor. Dünya koordinatları
    (z=0.004, x~-10) verilince sessizce saçma bir poz dönüyordu -- çökme yok,
    sadece 19 m'lik konum hatası. Ölçüldü: bu haliyle yeniden-yansıtma
    0.135 px, yanlış haliyle 50-15000 px.
    """
    h = size / 2.0
    return np.array([[-h, +h, 0.0], [+h, +h, 0.0],
                     [+h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def solve_pose(corners, ids, tags, size, K, dist, max_reproj: float):
    """Her tag'i kendi çerçevesinde çözüp dünyaya taşır, sonuçları
    ortalar. `(kamera_dünya_xyz, burun_yönü, ortalama_yeniden_yansıtma)`
    ya da None.

    Tag'in kendi çerçevesi dünya eksenleriyle çakışık: markörler zemine
    rpy=0 ile serildiği ve doku ters çevrilmediği için ek bir dönüş yok
    (ölçüldü: dört olası çeyrek dönüşten yalnızca 0° yaw'ı doğru veriyor).
    """
    obj = tag_object_points(size)
    positions, noses, reps = [], [], []
    for c, i in zip(corners, ids.ravel()):
        if i not in tags:
            continue                    # haritada olmayan tag -- yok say
        ip = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, ip, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        rep = float(np.linalg.norm(proj.reshape(4, 2) - ip, axis=1).mean())
        # DİKKAT: `not (rep <= max)` biçimi bilinçli. `rep > max` yazmak
        # NaN'ı geçirir (NaN ile her karşılaştırma False) ve tek bir bozuk
        # kare bütün istatistiği NaN yapıyordu.
        if not (rep <= max_reproj):
            # Kötü oturan çözümü sessizce kabul etmektense atmak yeğ:
            # yukarıdaki hata tam da böyle bir çözümdü.
            continue
        R, _ = cv2.Rodrigues(rvec)      # tag -> kamera (= dünya -> kamera)
        p = tags[i] + (-R.T @ tvec).ravel()
        if not np.all(np.isfinite(p)):
            continue
        positions.append(p)
        noses.append(-R[1])             # gövde +X'in dünyadaki yönü
        reps.append(rep)
    if not positions:
        return None
    return (np.mean(positions, axis=0), np.mean(noses, axis=0),
            float(np.mean(reps)))


def vehicle_from_camera(cam_world: np.ndarray, nose: np.ndarray,
                        cam_offset: np.ndarray) -> tuple[np.ndarray, float]:
    """Kamera pozundan aracın gövde pozu ve dünya yaw'ı.

    Kameranın optik çerçevesi (OpenCV: x sağ, y aşağı, z ileri) gövdeye sabit
    bağlı. Alt kamera base_link'e göre pitch +90° ile duruyor; bu, optik
    eksenleri gövdeye şöyle oturtuyor:
        x_optik = gövde -Y (sağ),  y_optik = gövde -X (geri),  z_optik = aşağı
    Dönme matrisinin satırları optik eksenlerin DÜNYA'daki yönleri; burnun
    dünya yönü bu yüzden -y_optik (`solve_pose` bunu döndürüyor).
    Doğrulandı: gerçek yaw 0 iken hesaplanan yaw +0.7°.
    """
    yaw = math.atan2(nose[1], nose[0])
    body_x = np.array([nose[0], nose[1], 0.0])
    n = np.linalg.norm(body_x)
    body_x = body_x / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    body_z = np.array([0.0, 0.0, 1.0])
    body_y = np.cross(body_z, body_x)
    R_wb = np.column_stack([body_x, body_y, body_z])
    return cam_world - R_wb @ cam_offset, yaw


# --------------------------------------------------------------------------
# Raf (dikey) AprilTag yolu -- ÖN kamera. Floor yolu (alt kamera) yukarıda,
# dokunulmadı. Fark: raf tag'leri dikey, her birinin dünya yönelimi farklı,
# ve ön kamera montajı alt kameradan başka. Genel PnP kullanılıyor.
# --------------------------------------------------------------------------

# Ön kamera OpenCV optik çerçevesi -> gövde (FLU: x ileri, y sol, z yukarı).
# Ön kamera rpy=0, gz optik ekseni +X (ileri): x_opencv=-Y_gövde (sağ),
# y_opencv=-Z_gövde (aşağı), z_opencv=+X_gövde (ileri).
FRONT_BODY_FROM_CAM = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                               dtype=np.float64)


def rot_gz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """gz/SDF rpy -> dönme matrisi (Rz@Ry@Rx). Tag'in KENDİ çerçevesini
    dünyaya taşır. Floor tag rpy=0 -> I (mevcut floor yolunun örtük varsayımı);
    raf tag'i AYNI doku/mesh'in döndürülmüşü olduğu için R_tw=Rot(rpy) -- bu
    sayede floor'un ampirik aruco köşe kalibrasyonu raf'a da geçiyor, 4-katlı
    köşe belirsizliği yeniden çözülmek zorunda kalmıyor."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_rack_tags(gt_path: Path, cfg: dict):
    """`id -> (tag MERKEZİ dünya (3,), R_tw (3,3))` ve tag kenarı. Raf tag'i
    yoksa boş döner. Merkez, etiket içi ofset (dx,dy) tag düzleminde R_tw ile
    taşınarak bulunur (caption yok -> ofset ~0)."""
    codes = cfg["codes"]
    if "rack_marker" not in codes:
        return {}, 0.0
    size, dx, dy = gl.floor_marker_geometry(
        codes["rack_marker"], codes["texture_px_per_m"], codes["max_texture_px"])
    out: dict[int, tuple] = {}
    for c in json.loads(gt_path.read_text())["codes"]:
        if c["type"] != "rack_marker":
            continue
        tid = int(c["payload"].rsplit(":", 1)[1])
        x, y, z, roll, pitch, yaw = c["label_pose_xyzrpy"]
        R = rot_gz(roll, pitch, yaw)
        center = np.array([x, y, z], dtype=np.float64) + R @ np.array([dx, dy, 0.0])
        out[tid] = (center, R)
    return out, size


def solve_rack(corners, ids, rack_tags, size, K, dist, cam_offset,
               max_reproj: float):
    """Ön kameradan dikey raf tag'leriyle aracın GÖVDE pozu + dünya yaw'ı.
    Genel PnP: her tag kendi çerçevesinde çözülür (IPPE_SQUARE, z=0 düzlemi),
    bilinen dünya pozuyla (t_tw, R_tw) dünyaya, sonra ön kamera montajıyla
    gövdeye taşınır. `(pos_dünya(3,), yaw, ort_reproj)` ya da None. Her çözüme
    yeniden-yansıtma kapısı (floor yolundaki ders)."""
    obj = tag_object_points(size)
    poss, noses, reps = [], [], []
    for c, i in zip(corners, ids.ravel()):
        if i not in rack_tags:
            continue
        ip = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, ip, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        rep = float(np.linalg.norm(proj.reshape(4, 2) - ip, axis=1).mean())
        if not (rep <= max_reproj):
            continue
        R_ct, _ = cv2.Rodrigues(rvec)              # tag -> kamera
        t_tw, R_tw = rack_tags[i]
        C_w = t_tw + R_tw @ (-R_ct.T @ tvec).ravel()   # kamera merkezi (dünya)
        R_wc = R_tw @ R_ct.T                           # kamera -> dünya
        R_wb = R_wc @ FRONT_BODY_FROM_CAM.T            # gövde -> dünya
        p = C_w - R_wb @ cam_offset
        if not np.all(np.isfinite(p)):
            continue
        poss.append(p)
        noses.append(R_wb[:, 0])                   # gövde +X'in dünya yönü
        reps.append(rep)
    if not poss:
        return None
    nose = np.mean(noses, axis=0)
    return (np.mean(poss, axis=0), math.atan2(nose[1], nose[0]),
            float(np.mean(reps)))


def send_vpe(link, spawn, pos, yaw, stamp) -> None:
    """Aracın dünya pozunu PX4'e VISION_POSITION_ESTIMATE olarak besler
    (alt ve ön kamera yolları ortak kullanır). NED: Kuzey=dünya+Y,
    Doğu=dünya+X, Aşağı=-Z. Zaman damgası görüntünün YAKALAMA anı (sim-zamanı,
    header.stamp), gönderme anının duvar saati DEĞİL -- duvar saati faz
    gecikmesi olarak girip seyrek düzeltmede salınım besliyordu (07_18_14.ulg).
    Damga boşsa duvar saatine düşer (sessiz ts=0 göndermektense)."""
    n = pos[1] - spawn[1]
    e = pos[0] - spawn[0]
    d = -(pos[2] - 0.0)
    heading = math.pi / 2 - yaw                    # dünya yaw -> NED heading
    ts = int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000
    if ts <= 0:
        ts = int(time.time() * 1e6)
    link.mav.vision_position_estimate_send(
        ts, float(n), float(e), float(d), 0.0, 0.0, float(heading))


def self_test_rack(cfg: dict, gt_path: Path) -> int:
    """Offline frame-cebiri doğrulaması (ROS/PX4 GEREKMEZ). Bilinen bir araç
    pozundan bir raf tag'ini ön kameraya yansıt, solve_rack ile geri çöz;
    poz geri kurtarılıyorsa cebir doğru. Uçamadan önceki tek güvence budur --
    kalan risk (aruco'nun dokuyu okuma yönü) floor kalibrasyonundan miras."""
    front = next(c for c in cfg["cameras"] if c["name"] == "front")
    K, dist = intrinsics(front)
    cam_off = np.array(front["pose"][:3], dtype=np.float64)
    rack, size = load_rack_tags(gt_path, cfg)
    if not rack:
        print("raf tag'i yok (world yeniden üretilmeli).", file=sys.stderr)
        return 1
    # A satırı (koridor 1'e bakar, y<-4), seviye 2 (z~2.09) bir tag
    cand = [i for i, (c, R) in rack.items() if c[1] < -4 and abs(c[2] - 2.09) < 0.05]
    ok_all = True
    for tid in cand[:3]:
        t_tw, R_tw = rack[tid]
        # bilinen poz: koridor 1 merkezi, tag'in karşısı, A'ya bakıyor (yaw -90)
        pos = np.array([t_tw[0], -3.40, 2.09]); yaw = -math.pi / 2
        nose = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        R_wb = np.column_stack([nose, np.cross([0, 0, 1.0], nose), [0, 0, 1.0]])
        C_w = pos + R_wb @ cam_off
        R_wc = R_wb @ FRONT_BODY_FROM_CAM
        obj = tag_object_points(size)
        Xw = t_tw + (R_tw @ obj.T).T                     # köşeler dünya
        Xc = (R_wc.T @ (Xw - C_w).T).T                   # kameraya
        ip = np.column_stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2],
                              K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]])
        res = solve_rack([ip.reshape(1, 4, 2)], np.array([[tid]]), rack,
                         size, K, dist, cam_off, 2.0)
        if res is None:
            print(f"  tag {tid}: solve_rack None"); ok_all = False; continue
        rp, ry, rep = res
        dp = float(np.linalg.norm(rp - pos)) * 1000
        dy = math.degrees(abs((ry - yaw + math.pi) % (2 * math.pi) - math.pi))
        good = dp < 1.0 and dy < 0.01
        ok_all &= good
        print(f"  tag {tid} @ ({t_tw[0]:+.1f},{t_tw[1]:+.1f},{t_tw[2]:.2f}): "
              f"konum hatası {dp:.4f} mm, yaw hatası {dy:.5f}°, rep {rep:.3f} px  "
              f"{'GEÇTİ' if good else 'KALDI'}")
    print("SELF-TEST:", "GEÇTİ (frame cebiri tutarlı)" if ok_all
          else "KALDI -- frame cebiri yanlış, uçmadan düzelt")
    return 0 if ok_all else 1


class TruthReader:
    """SADECE --compare için: Gazebo'nun gerçek pozu. Kestirime girmez."""

    def __init__(self, world: str, model: str):
        import os
        self.model, self.pose, self.yaw = model, None, None
        env = {**os.environ,
               "PATH": f"{PROJECT_ROOT / 'scripts' / 'bin'}:" + os.environ["PATH"]}
        self._p = subprocess.Popen(
            ["gz", "topic", "-e", "-t", f"/world/{world}/dynamic_pose/info",
             "--json-output"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, env=env)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        dec, buf = json.JSONDecoder(), ""
        for line in self._p.stdout:
            buf += line
            while buf.strip():
                try:
                    o, end = dec.raw_decode(buf.lstrip())
                except ValueError:
                    break
                buf = buf.lstrip()[end:]
                for p in o.get("pose", []):
                    if p.get("name") == self.model:
                        pos, q = p.get("position", {}), p.get("orientation", {})
                        self.pose = np.array([pos.get("x", 0.0), pos.get("y", 0.0),
                                              pos.get("z", 0.0)])
                        qw, qx = q.get("w", 0.0), q.get("x", 0.0)
                        qy, qz = q.get("y", 0.0), q.get("z", 0.0)
                        self.yaw = math.atan2(2 * (qw * qz + qx * qy),
                                              1 - 2 * (qy * qy + qz * qz))

    def stop(self):
        self._p.terminate()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--front-topic", default=FRONT_TOPIC,
                    help="raf tag'leri için ön kamera konusu")
    ap.add_argument("--no-rack", action="store_true",
                    help="raf tag'lerini (ön kamera) kullanma, sadece floor")
    ap.add_argument("--self-test", action="store_true",
                    help="offline frame-cebiri testini koş ve çık (ROS/PX4 gerekmez)")
    ap.add_argument("--url", default="udpin:0.0.0.0:14550",
                    help="PX4 MAVLink. 14540 uçuş betiklerinde kullanılıyor.")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="PX4'e gönderme, sadece hesapla ve raporla")
    ap.add_argument("--compare", action="store_true",
                    help="Gazebo gerçek pozuyla karşılaştır (yalnızca ölçüm)")
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--no-bridge", action="store_true")
    ap.add_argument("--max-reproj", type=float, default=2.0,
                    help="bu piksel değerinden kötü oturan çözümü at (kapı)")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--model", default="warehouse_scout_0")
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    if args.self_test:
        return self_test_rack(cfg, args.ground_truth)

    cam = next(c for c in cfg["cameras"] if c["name"] == "down")
    K, dist = intrinsics(cam)
    cam_offset = np.array(cam["pose"][:3], dtype=np.float64)
    tags, tag_size = load_tag_map(args.ground_truth, cfg)
    spawn = np.array(cfg["spawn"]["pose"][:3], dtype=np.float64)
    print(f"{len(tags)} floor tag (kenar {tag_size:.4f} m), "
          f"alt kamera fx={K[0,0]:.1f} px")
    print(f"yerel origin (config spawn): {spawn[0]:.2f}, {spawn[1]:.2f}")

    # Raf tag'leri (ön kamera). Floor'la aynı EKF konusuna beslenir; ön kamera
    # tarama boyunca sürekli tag gördüğü için floor'un 4 m boşluğunu kapatır.
    fcam = next(c for c in cfg["cameras"] if c["name"] == "front")
    fK, fdist = intrinsics(fcam)
    fcam_offset = np.array(fcam["pose"][:3], dtype=np.float64)
    rack_tags, rack_size = load_rack_tags(args.ground_truth, cfg)
    use_rack = bool(rack_tags) and not args.no_rack
    if use_rack:
        print(f"{len(rack_tags)} raf tag (kenar {rack_size:.4f} m), "
              f"ön kamera fx={fK[0,0]:.1f} px")
    elif not args.no_rack:
        print("raf tag YOK (world eski) -- sadece floor.")

    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters())

    truth = TruthReader(args.world, args.model) if args.compare else None
    link = None
    if not args.dry_run:
        link = mavutil.mavlink_connection(args.url, source_system=1,
                                          source_component=197)
        print(f"PX4 bekleniyor ({args.url})...")
        link.wait_heartbeat(timeout=30)
        print(f"  bağlandı: sistem {link.target_system}")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    stats = {"frames": 0, "fixes": 0, "err": [], "yaw_err": [], "rep": [],
             "rack_frames": 0, "rack_fixes": 0, "rack_rep": []}

    class LocNode(Node):
        def __init__(self):
            super().__init__("apriltag_localizer")
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, args.topic, self.on_image, qos)
            if use_rack:
                self.create_subscription(Image, args.front_topic,
                                         self.on_front, qos)
            self.last_report = 0.0
            self.last_rack_report = 0.0

        def on_front(self, msg: Image) -> None:
            """Ön kameradan dikey raf tag'leriyle lokalizasyon. Floor yolundan
            (on_image) bağımsız çalışır, aynı EKF konusuna besler; ikisi de
            reprojeksiyon kapısıyla korunuyor."""
            stats["rack_frames"] += 1
            gray = to_gray(to_array(msg))
            corners, ids, _ = det.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                return
            res = solve_rack(corners, ids, rack_tags, rack_size, fK, fdist,
                             fcam_offset, args.max_reproj)
            if res is None:
                return
            pos, yaw, rep = res
            stats["rack_fixes"] += 1
            stats["rack_rep"].append(rep)
            if link is not None:
                send_vpe(link, spawn, pos, yaw, msg.header.stamp)
            now = time.time()
            if now - self.last_rack_report >= 3.0:
                self.last_rack_report = now
                line = (f"RAF tag {sorted(int(i) for i in ids.ravel())}  "
                        f"konum ({pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:5.2f})  "
                        f"yaw {math.degrees(yaw):6.1f}°  rep {rep:4.2f}px")
                if truth is not None and truth.pose is not None:
                    err = float(np.linalg.norm(pos[:2] - truth.pose[:2]))
                    line += f"   hata {err*100:5.1f} cm"
                print(line)

        def on_image(self, msg: Image) -> None:
            stats["frames"] += 1
            gray = to_gray(to_array(msg))
            corners, ids, _ = det.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                return
            res = solve_pose(corners, ids, tags, tag_size, K, dist, args.max_reproj)
            if res is None:
                return
            cam_world, nose, rep = res
            pos, yaw = vehicle_from_camera(cam_world, nose, cam_offset)
            stats["fixes"] += 1
            stats["rep"].append(rep)

            if link is not None:
                send_vpe(link, spawn, pos, yaw, msg.header.stamp)

            if truth is not None and truth.pose is not None:
                err = float(np.linalg.norm(pos[:2] - truth.pose[:2]))
                ye = math.degrees((yaw - truth.yaw + math.pi) % (2 * math.pi) - math.pi)
                stats["err"].append(err)
                stats["yaw_err"].append(ye)

            now = time.time()
            if now - self.last_report >= 3.0:
                self.last_report = now
                line = (f"tag {sorted(ids.ravel().tolist())}  "
                        f"konum ({pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:5.2f})  "
                        f"yaw {math.degrees(yaw):6.1f}°  rep {rep:4.2f}px")
                if stats["err"]:
                    line += (f"   hata {stats['err'][-1]*100:5.1f} cm / "
                             f"{stats['yaw_err'][-1]:+5.1f}°")
                print(line)

    bridges = []
    if not args.no_bridge:
        topics = [args.topic] + ([args.front_topic] if use_rack else [])
        for t in topics:
            bridges.append(subprocess.Popen(
                ["ros2", "run", "ros_gz_image", "image_bridge", t],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        time.sleep(4.0)

    rclpy.init()
    node = LocNode()
    listening = args.topic + (f" + {args.front_topic}" if use_rack else "")
    print(f"dinleniyor: {listening}"
          f"{'  (KURU KOŞU -- PX4''e gönderilmiyor)' if args.dry_run else ''}")
    deadline = time.time() + args.duration if args.duration else None
    try:
        while rclpy.ok() and not (deadline and time.time() >= deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for b in bridges:
            b.terminate()
        if truth:
            truth.stop()

    print(f"\nfloor (alt kamera): {stats['frames']} kare, {stats['fixes']} poz "
          f"({100*stats['fixes']/max(1,stats['frames']):.0f}%)")
    if stats["rep"]:
        r = np.array(stats["rep"])
        print(f"  yeniden-yansıtma: ortalama {r.mean():.3f} px, "
              f"en fazla {r.max():.3f} px")
    if use_rack:
        print(f"raf (ön kamera)   : {stats['rack_frames']} kare, "
              f"{stats['rack_fixes']} poz "
              f"({100*stats['rack_fixes']/max(1,stats['rack_frames']):.0f}%)")
        if stats["rack_rep"]:
            r = np.array(stats["rack_rep"])
            print(f"  yeniden-yansıtma: ortalama {r.mean():.3f} px, "
                  f"en fazla {r.max():.3f} px")
    if stats["err"]:
        e = np.array(stats["err"])
        y = np.array(stats["yaw_err"])
        print(f"konum hatası      : ortalama {e.mean()*100:.1f} cm, "
              f"en fazla {e.max()*100:.1f} cm")
        print(f"yaw hatası        : ortalama {y.mean():+.2f}°, "
              f"en fazla {np.abs(y).max():.2f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
