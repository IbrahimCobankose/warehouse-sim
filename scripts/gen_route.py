#!/usr/bin/env python3
"""Tarama rotasını DEPO GEOMETRİSİNDEN türetir (config/warehouse.yaml).

Neden: eski rota (config/route.yaml) waypoint'leri AprilTag konumlarına
bağlıydı (x = -10,-6,-2,+2,+6) ve seviye geçişlerini rafın uç bacaklarına
"ramp" olarak gömüyordu. İki sorun çıkıyordu:
  1) Her seviyede bir UÇ GÖZ, sabit irtifada değil irtifa değişirken taranıyordu
     -> o göz temiz taranmıyordu ("level'i sonuna kadar taramıyor").
  2) Ramp/tag bağımlılığı L1'de kırılgandı.

Çözüm (kullanıcı yönlendirmesi: "depoyu biliyoruz, raf ölçülerini drone'a
veririz"): raf başlangıcı/uzunluğu/yükseklikleri BİLİNİYOR. Rotayı bu ölçülerden
türet:
  * Her raf yüzü, her seviyede rafın GERÇEK ucundan ucuna (x_origin ..
    x_origin+bay_width*bay_count) TEK SABİT İRTİFADA taranır -> tam kapsama.
  * BÜTÜN irtifa değişimleri rafın bittiği AÇIK ALANDA yapılır (köşe
    x = ±turnaround), öteleyerek (ramp) -- rafın önünde asla irtifa değişmez,
    böylece hiçbir göz feda edilmez ve tarama karesi (rack tag) hep ortada kalır.

Localization DEĞİŞMEZ: apriltag_localize.py bağımsız çalışıp kameraların
gördüğü tag'lerden PX4'e VPE besler; rota tag adı vermese de tarama boyunca ön
kamera raf tag'lerini görür. Açık-alan köşelerinde alt kamera floor turnaround
tag'lerini görür (config'de x=±8.5/12.3'te tanımlı).

    .venv/bin/python scripts/gen_route.py           # -> config/route_gen.yaml
    .venv/bin/python scripts/fly_route.py tam_tur_geo --routes config/route_gen.yaml --dry-run

Not: bu üretici L1 lokalizasyon kök-nedenini ÇÖZMEZ (o ayrı iş kolu); temiz
kapsamayı sağlar ve L1 maruziyetini (ramp-sırasında-tarama) ortadan kaldırır.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def rnd(v: float) -> float:
    return round(float(v), 2)


def build(cfg: dict) -> dict:
    r = cfg["racking"]
    xs = r["x_origin"]
    xe = r["x_origin"] + r["bay_width"] * r["bay_count"]          # raf uçları
    turn = cfg["codes"]["floor_marker"]["turnarounds"]
    xcp = max(t[0] for t in turn)                                 # açık köşe +x
    xcn = min(t[0] for t in turn)                                 # açık köşe -x
    L1, L2, L3 = cfg["codes"]["rack_marker"]["heights"]           # seviye irtifaları
    ais = {a["id"]: a["y_center"] for a in r["aisles"]}
    y1, y2 = ais[1], ais[2]

    # Her raf yüzü: koridordan bakış yönü (world yaw, derece).
    # facing +1 -> ürün yüzü +y'ye bakar -> koridor +y'deyse drone -y'ye (yaw-90) bakar.
    # Pratikte: yüz koridorun HANGİ tarafındaysa drone o tarafa döner.
    def look_yaw(y0: float, aisle_y: float) -> int:
        return -90 if y0 < aisle_y else 90        # yüz -y'deyse -y'ye bak, değilse +y

    rows = {row["id"]: row for row in r["rows"]}

    ORDER = [L2, L3, L1]     # seviye sırası: L2->L3->L1 (riskli L1 en sonda, kanıtlı)

    tour: list = []

    def scan(x, y, yaw, alt, **kw):
        wp = {"xy": [rnd(x), rnd(y)], "yaw": int(yaw), "alt": rnd(alt)}
        wp.update(kw)
        tour.append(wp)

    def add_face(face_id, y, yaw, start_side, entry_from_corner, aisle):
        """Bir raf yüzünü 3 seviyede tarar (snake). Her seviye rafın ucundan
        ucuna TEK irtifada; seviye geçişi açık köşede öteleyerek (ramp).
        start_side: 'neg'(xs'ten başla) / 'pos'(xe'den). entry_from_corner:
        önceki connector bizi zaten köşeye/irtifaya koydu mu (ilk xstart'ı atla).
        Döner: yüz bittiğinde bulunulan taraf ('neg'/'pos')."""
        side = start_side
        for k, lvl in enumerate(ORDER):
            xstart = xs if side == "neg" else xe
            xend = xe if side == "neg" else xs
            dirn = "ileri(+x)" if xend == xe else "geri(-x)"
            lvlname = {L1: "L1", L2: "L2", L3: "L3"}[lvl]
            if k == 0 and not entry_from_corner:
                scan(xstart, y, yaw, lvl,
                     log=f"{face_id} {lvlname} {dirn} -- yüz başı")
            scan(xend, y, yaw, lvl,
                 log=f"{face_id} {lvlname} tarandı ({dirn})")
            if k < len(ORDER) - 1:
                cx = xcp if xend == xe else xcn
                nxt = ORDER[k + 1]
                nxtname = {L1: "L1", L2: "L2", L3: "L3"}[nxt]
                scan(cx, y, yaw, nxt, ramp=True,
                     log=f"açık köşe -- öteleyerek {nxtname}'e geç (raf önünde DEĞİL)")
            side = "pos" if side == "neg" else "neg"
        return side

    def connector_spin(y, cur_yaw, new_yaw, end_side, new_first_level):
        """Aynı koridorda yüz->yüz (A->B, D->C): bitiş köşesinde HOVER'da
        180° dön + öteleyerek yeni yüzün ilk seviyesine (L2) çık. Dönüş açık
        alanda ve yüksekte (L2), raf yanında asla."""
        cx = xcp if end_side == "pos" else xcn
        scan(cx, y, cur_yaw, new_first_level, ramp=True, spin=int(new_yaw),
             log=f"açık köşe -- {int(new_yaw):+d}° dön + L2'ye çık (sonraki yüze)")

    def connector_cross(cur_yaw, from_y, to_y, next_yaw):
        """Koridor->koridor geçişi (-x açık ucundan): burnu -x'e çevir, köşeye
        çık, +y'ye geçip diğer koridora, öteleyerek L2'ye. Mevcut negx dönüşünün
        geometriden türetilmişi."""
        scan(xcn, from_y, cur_yaw, L1, spin=180,
             log="-x köşeye çık, burnu rafın -x ucuna")
        scan(xcn, to_y, 180, L2, ramp=True, spin=int(next_yaw),
             log="diğer koridora geç + L2'ye çık")

    # ---- KORİDOR 1: A yüzü, sonra B yüzü ----
    yaw_A = look_yaw(rows["A"]["y0"], y1)
    yaw_B = look_yaw(rows["B"]["y0"], y1)
    end_side = add_face("A", y1, yaw_A, "neg", entry_from_corner=False, aisle=y1)
    connector_spin(y1, yaw_A, yaw_B, end_side, ORDER[0])
    end_side = add_face("B", y1, yaw_B, end_side, entry_from_corner=True, aisle=y1)

    kor1_end = len(tour)         # koridor 1 burada biter (alt-rota için)

    # ---- KORİDOR 1 -> 2 geçişi ----
    connector_cross(yaw_B, y1, y2, look_yaw(rows["D"]["y0"], y2))

    # ---- KORİDOR 2: D yüzü, sonra C yüzü ----
    yaw_D = look_yaw(rows["D"]["y0"], y2)
    yaw_C = look_yaw(rows["C"]["y0"], y2)
    end_side = add_face("D", y2, yaw_D, "neg", entry_from_corner=True, aisle=y2)
    connector_spin(y2, yaw_D, yaw_C, end_side, ORDER[0])
    end_side = add_face("C", y2, yaw_C, end_side, entry_from_corner=True, aisle=y2)

    # Son waypoint: bekleme + tamam logu.
    tour[-1]["log"] = "TAM TUR TAMAM -- 2 koridor x 2 yüz x 3 seviye"
    tour[-1]["hold"] = 3.0

    # Alt-rota: sadece koridor 1 (A+B) -- izole doğrulama için. deepcopy: sığ
    # kopya xy listelerini iki rota arasında paylaştırıp YAML anchor üretiyordu.
    import copy
    kor1 = copy.deepcopy(tour[:kor1_end])
    kor1[-1]["log"] = "KORİDOR 1 TAMAM (A+B, 3 seviye)"
    kor1[-1]["hold"] = 3.0
    # kor1'in son waypoint'inden sonraki 'ramp/spin' başka yüze aitti -> yok.

    common = dict(
        speed=0.7, waypoint_tolerance=0.35, slow_radius=1.0, slow_floor=0.2,
        # l1_speed 0.4->0.25: L1 en kırılgan rejim; yavaş uçuş kare başına daha
        # çok raf-tag örtüşmesi -> EKF beslemesi ayakta kalır (diverjans olasılığı
        # düşer). NOT: EKF zaten saptıysa bunu SINIRLAMAZ (carrot EKF-uzayında);
        # bu bir önlem, kök çözüm değil -- EV log kök nedeni gösterecek.
        lookahead=0.8, l1_speed=0.25, l1_alt=0.9)

    def route(waypoints):
        return {"aisle": 1, "yaw": -90, "altitude": L2,
                "waypoints": waypoints}

    return {
        **common,
        "routes": {
            "koridor1_geo": route(kor1),
            "tam_tur_geo": route(tour),
        },
    }


HEADER = """\
# OTOMATİK ÜRETİLDİ -- elle düzenleme; kaynak: scripts/gen_route.py
#   .venv/bin/python scripts/gen_route.py
#
# Rota depo geometrisinden (config/warehouse.yaml) türetilir:
#   * Her raf yüzü her seviyede rafın ucundan ucuna TEK SABİT İRTİFADA taranır.
#   * BÜTÜN irtifa değişimleri açık köşede (x=±turnaround) ÖTELEYEREK yapılır --
#     raf önünde asla, böylece hiçbir uç göz feda edilmez.
# Waypoint biçimi fly_route.py ile aynı: {xy,yaw,alt,ramp?,spin?,log?,hold?}.
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path,
                    default=PROJECT_ROOT / "config" / "route_gen.yaml")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    data = build(cfg)
    text = HEADER + yaml.safe_dump(data, allow_unicode=True, sort_keys=False,
                                   default_flow_style=None, width=100)
    args.out.write_text(text)
    n1 = len(data["routes"]["koridor1_geo"]["waypoints"])
    nt = len(data["routes"]["tam_tur_geo"]["waypoints"])
    print(f"yazıldı: {args.out}")
    print(f"  koridor1_geo : {n1} nokta")
    print(f"  tam_tur_geo  : {nt} nokta")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
