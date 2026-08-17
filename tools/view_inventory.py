#!/usr/bin/env python3
"""3B envanter görselleştirmesi: inventory.json -> tek dosyalık HTML.

    .venv/bin/python tools/view_inventory.py
    .venv/bin/python tools/view_inventory.py --truth      # hata vektörleriyle
    firefox out/inventory_3d.html

Amaç (sprint B7, MVP): kestirim hattının her ürünü doğru raf koordinatına
YAZDIĞINI gözle doğrulamak. 216 satırlık tabloda gözden kaçan bir hata,
rafın dışına düşmüş tek bir nokta olarak bakışta görünür.

Gösterilenler:
  * depo koordinat sistemi (dünya eksenleri + zemin ızgarası)
  * raf yüzleri, gözler ve seviyeler (tel kafes)
  * okunan ürünler (nokta; güvene göre renk)
  * okunamayan/kaçan ürünler (ayrı renk ve şekil)
  * imleç bir ürünün üstündeyken kimlik, QR, koordinat, güven

DIŞ BAĞIMLILIK YOK: küçük bir perspektif projeksiyon + canvas çizimi gömülü.
three.js gibi bir kütüphane CDN'den gelirdi ve çevrimdışı açılmazdı; bu dosya
e-postayla gönderilse bile çalışır.

GROUND TRUTH yalnızca --truth ile ve yalnızca GÖSTERİM için okunur (kaçan
ürünlerin nerede olduğu + hata vektörü). Kestirime girmez.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def rack_boxes(cfg: dict) -> list[dict]:
    """Her (yüz, göz, seviye) için tel kafes kutu: merkez + yarı boyutlar."""
    r = cfg["racking"]
    out = []
    for row in r["rows"]:
        face = row["y0"] + r["depth"] if row["facing"] > 0 else row["y0"]
        y_c = face - row["facing"] * r["depth"] / 2.0      # blok merkezi
        for bay in range(1, r["bay_count"] + 1):
            x0 = r["x_origin"] + (bay - 1) * r["bay_width"]
            for li, h in enumerate(r["level_heights"], start=1):
                out.append({
                    "row": row["id"], "bay": bay, "level": li,
                    "c": [x0 + r["bay_width"] / 2.0, y_c, h + 0.30],
                    "h": [r["bay_width"] / 2.0, r["depth"] / 2.0, 0.30],
                })
    return out


HTML = r"""<!doctype html>
<meta charset="utf-8"><title>3B Envanter — WareDrone</title>
<style>
  html,body{margin:0;height:100%;background:#11151c;color:#dfe6ef;
            font-family:system-ui,sans-serif;overflow:hidden}
  canvas{display:block;width:100vw;height:100vh;cursor:grab}
  canvas.drag{cursor:grabbing}
  #hud{position:fixed;top:12px;left:14px;font-size:13px;line-height:1.5;
       background:#0009;padding:10px 13px;border-radius:7px;max-width:330px}
  #hud b{font-size:15px}
  #tip{position:fixed;pointer-events:none;background:#000d;border:1px solid #4a5568;
       padding:7px 10px;border-radius:6px;font-size:12.5px;line-height:1.45;
       display:none;white-space:nowrap}
  .k{display:inline-block;width:11px;height:11px;border-radius:50%;
     margin-right:6px;vertical-align:-1px}
  /* kaçan ürün çizimde içi boş KARE; efsane de aynı şekli göstersin,
     yoksa "düşük güven" kırmızısıyla karışıyor */
  .sq{border-radius:0;background:none!important;border:2px solid #ff4d6d}
  #help{position:fixed;bottom:10px;left:14px;font-size:12px;color:#8b98a9}
</style>
<canvas id="c"></canvas>
<div id="hud"></div><div id="tip"></div>
<div id="help">sürükle: döndür &nbsp;·&nbsp; tekerlek: yakınlaştır &nbsp;·&nbsp;
  sağ tık sürükle: kaydır &nbsp;·&nbsp; imleci ürünün üstüne getir</div>
<script>
const D = __DATA__;

// ---------------------------------------------------------------- kamera
let az = -0.9, el = 0.38, dist = 26, target = [-1.5, -0.8, 2.0];
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
let W, H, dpr;
function resize(){
  dpr = window.devicePixelRatio || 1;
  W = cv.clientWidth; H = cv.clientHeight;
  cv.width = W*dpr; cv.height = H*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  draw();
}
window.addEventListener('resize', resize);

/* Dünya (ENU: x doğu, y kuzey, z yukarı) -> kamera -> ekran.
   Kamera hedefin etrafında küresel; yukarı yön dünya +Z. */
function project(p){
  const ca=Math.cos(az), sa=Math.sin(az), ce=Math.cos(el), se=Math.sin(el);
  const eye=[target[0]+dist*ce*ca, target[1]+dist*ce*sa, target[2]+dist*se];
  const f=[target[0]-eye[0], target[1]-eye[1], target[2]-eye[2]];
  const fl=Math.hypot(...f); const fw=f.map(v=>v/fl);
  // sağ = ileri x yukarı(0,0,1); yukarı = sağ x ileri
  let r=[fw[1], -fw[0], 0]; const rl=Math.hypot(...r)||1; r=r.map(v=>v/rl);
  const u=[r[1]*fw[2]-r[2]*fw[1], r[2]*fw[0]-r[0]*fw[2], r[0]*fw[1]-r[1]*fw[0]];
  const d=[p[0]-eye[0], p[1]-eye[1], p[2]-eye[2]];
  const z=d[0]*fw[0]+d[1]*fw[1]+d[2]*fw[2];
  if(z<=0.05) return null;                       // kameranın arkasında
  const x=d[0]*r[0]+d[1]*r[1]+d[2]*r[2];
  const y=d[0]*u[0]+d[1]*u[1]+d[2]*u[2];
  const fov=Math.min(W,H)*0.9;
  return [W/2 + fov*x/z, H/2 - fov*y/z, z];
}
function line(a,b,style,w){
  const p=project(a), q=project(b); if(!p||!q) return;
  ctx.strokeStyle=style; ctx.lineWidth=w||1;
  ctx.beginPath(); ctx.moveTo(p[0],p[1]); ctx.lineTo(q[0],q[1]); ctx.stroke();
}
function boxEdges(c,h){
  const [x,y,z]=c, [a,b,d]=h;
  const v=[[x-a,y-b,z-d],[x+a,y-b,z-d],[x+a,y+b,z-d],[x-a,y+b,z-d],
           [x-a,y-b,z+d],[x+a,y-b,z+d],[x+a,y+b,z+d],[x-a,y+b,z+d]];
  return [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],
          [0,4],[1,5],[2,6],[3,7]].map(e=>[v[e[0]],v[e[1]]]);
}

// güven -> renk (kırmızı = düşük, yeşil = yüksek)
function confColor(c){
  const t=Math.max(0,Math.min(1,c));
  const r=Math.round(230*(1-t)+60*t), g=Math.round(70*(1-t)+200*t);
  return `rgb(${r},${g},90)`;
}

let screenPts=[];
function draw(){
  ctx.fillStyle='#11151c'; ctx.fillRect(0,0,W,H);

  // zemin ızgarası (2 m)
  for(let x=D.bounds[0]; x<=D.bounds[1]+0.01; x+=2)
    line([x,D.bounds[2],0],[x,D.bounds[3],0],'#1e2836');
  for(let y=D.bounds[2]; y<=D.bounds[3]+0.01; y+=2)
    line([D.bounds[0],y,0],[D.bounds[1],y,0],'#1e2836');

  // dünya eksenleri (koordinat sistemi)
  line([0,0,0],[3,0,0],'#e05252',2.5);
  line([0,0,0],[0,3,0],'#52c65c',2.5);
  line([0,0,0],[0,0,3],'#5b8dfc',2.5);

  // raf gözleri
  for(const b of D.racks)
    for(const e of boxEdges(b.c,b.h)) line(e[0],e[1],'#33455e',1);

  // ürünler: uzaktan yakına (ressam algoritması)
  screenPts=[];
  const pts=[];
  for(const it of D.items){
    const p=project([it.x,it.y,it.z]); if(!p) continue;
    pts.push([p,it]);
  }
  pts.sort((a,b)=>b[0][2]-a[0][2]);
  for(const [p,it] of pts){
    const r=Math.max(2.0, 62/p[2]);
    if(D.truth && it.tx!==undefined && it.err>0.15){
      const q=project([it.tx,it.ty,it.tz]);      // hata vektörü
      if(q){ ctx.strokeStyle='#ffb020'; ctx.lineWidth=1.2;
             ctx.beginPath(); ctx.moveTo(p[0],p[1]); ctx.lineTo(q[0],q[1]); ctx.stroke(); }
    }
    if(it.missed){                                // kaçan: içi boş kare
      ctx.strokeStyle='#ff4d6d'; ctx.lineWidth=1.8;
      ctx.strokeRect(p[0]-r, p[1]-r, 2*r, 2*r);
    } else {
      ctx.fillStyle=confColor(it.conf);
      ctx.beginPath(); ctx.arc(p[0],p[1],r,0,7); ctx.fill();
    }
    screenPts.push([p[0],p[1],r,it]);
  }
  hud();
}

function hud(){
  const n=D.items.filter(i=>!i.missed).length, m=D.items.length-n;
  let s=`<b>3B Envanter</b><br>${n} okunan ürün`;
  if(m) s+=` · <span style="color:#ff4d6d">${m} kaçan</span>`;
  s+=`<br><span style="font-size:12px;color:#8b98a9">${D.source}</span><hr
      style="border:0;border-top:1px solid #2a3648;margin:7px 0">`;
  s+=`<span class="k" style="background:${confColor(0.9)}"></span>yüksek güven<br>`;
  s+=`<span class="k" style="background:${confColor(0.15)}"></span>düşük güven<br>`;
  if(m) s+=`<span class="k sq"></span>kaçan (ground truth)<br>`;
  if(D.truth) s+=`<span class="k" style="background:#ffb020"></span>hata vektörü (&gt;15 cm)<br>`;
  s+=`<span style="color:#e05252">■</span> X &nbsp;<span style="color:#52c65c">■</span> Y
      &nbsp;<span style="color:#5b8dfc">■</span> Z`;
  if(D.stats) s+=`<br><span style="font-size:12px;color:#8b98a9">${D.stats}</span>`;
  document.getElementById('hud').innerHTML=s;
}

// ------------------------------------------------------------- etkileşim
let drag=null;
cv.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,b:e.button};
  cv.classList.add('drag');});
window.addEventListener('mouseup',()=>{drag=null;cv.classList.remove('drag');});
cv.addEventListener('contextmenu',e=>e.preventDefault());
window.addEventListener('mousemove',e=>{
  if(drag){
    const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
    drag.x=e.clientX; drag.y=e.clientY;
    if(drag.b===2){                                  // kaydır
      const ca=Math.cos(az), sa=Math.sin(az), k=dist*0.0016;
      target[0]+= (sa*dx)*k; target[1]+= (-ca*dx)*k; target[2]+= dy*k;
    } else {
      az-=dx*0.006; el=Math.max(-1.45,Math.min(1.45, el+dy*0.006));
    }
    draw(); return;
  }
  // en yakın ürün -> ipucu
  const t=document.getElementById('tip');
  let best=null, bd=1e9;
  for(const [x,y,r,it] of screenPts){
    const d=Math.hypot(e.clientX-x, e.clientY-y);
    if(d<Math.max(7,r+4) && d<bd){ bd=d; best=it; }
  }
  if(!best){ t.style.display='none'; return; }
  let h=`<b>${best.id||'(çözülemedi)'}</b><br>${best.qr}<br>`
      + `${best.shelf}-${String(best.bay).padStart(2,'0')}-L${best.level}<br>`
      + `x ${best.x.toFixed(2)}  y ${best.y.toFixed(2)}  z ${best.z.toFixed(2)}`;
  if(best.missed) h+=`<br><span style="color:#ff4d6d">TARANMADI</span>`;
  else h+=`<br>güven ${best.conf.toFixed(2)} · ${best.n} okuma`;
  if(best.err!==undefined && !best.missed)
    h+=`<br>konum hatası ${(best.err*100).toFixed(1)} cm`;
  t.innerHTML=h; t.style.display='block';
  t.style.left=(e.clientX+14)+'px'; t.style.top=(e.clientY+12)+'px';
});
cv.addEventListener('wheel',e=>{
  e.preventDefault();
  dist=Math.max(4,Math.min(90, dist*(e.deltaY>0?1.1:0.9))); draw();
},{passive:false});
resize();
</script>
"""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path,
                    default=PROJECT_ROOT / "out" / "inventory.json")
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path,
                    default=PROJECT_ROOT / "out" / "inventory_3d.html")
    ap.add_argument("--truth", action="store_true",
                    help="kaçan ürünleri ve hata vektörlerini de göster "
                         "(ground truth SADECE gösterim için okunur)")
    ap.add_argument("--ground-truth", type=Path,
                    default=PROJECT_ROOT / "out" / "ground_truth.json")
    args = ap.parse_args()

    if not args.inventory.exists():
        print(f"envanter yok: {args.inventory}\n"
              f"önce: .venv/bin/python tools/build_inventory.py")
        return 1

    cfg = yaml.safe_load(args.config.read_text())
    doc = json.loads(args.inventory.read_text())
    truth = {}
    if args.truth and args.ground_truth.exists():
        truth = {c["payload"]: c for c in
                 json.loads(args.ground_truth.read_text())["codes"]
                 if c["type"] == "box_qr"}

    items, errs = [], []
    for it in doc["items"]:
        rec = {"id": it["product_id"], "qr": it["qr"],
               "x": it["estimated_x"], "y": it["estimated_y"],
               "z": it["estimated_z"], "shelf": it["shelf"],
               "level": it["level"], "bay": it.get("bay", 0),
               "conf": it["confidence"], "n": it["n_readings"], "missed": False}
        t = truth.get(it["qr"])
        if t:
            tx, ty, tz = t["label_pose_xyzrpy"][:3]
            e = ((rec["x"] - tx) ** 2 + (rec["y"] - ty) ** 2
                 + (rec["z"] - tz) ** 2) ** 0.5
            rec.update(tx=tx, ty=ty, tz=tz, err=round(e, 3))
            errs.append(e)
        items.append(rec)

    # Kaçanlar: ground truth'ta var, envanterde yok. Yalnız --truth ile.
    seen = {it["qr"] for it in doc["items"]}
    for payload, c in truth.items():
        if payload in seen:
            continue
        x, y, z = c["label_pose_xyzrpy"][:3]
        items.append({"id": payload.split("|")[-1], "qr": payload,
                      "x": x, "y": y, "z": z, "shelf": c["row"],
                      "level": c["level"], "bay": c["bay"],
                      "conf": 0.0, "n": 0, "missed": True})

    r = cfg["racking"]
    xs = [r["x_origin"] - 2, r["x_origin"] + r["bay_count"] * r["bay_width"] + 2]
    ys = [min(w["y0"] for w in r["rows"]) - 2,
          max(w["y0"] for w in r["rows"]) + r["depth"] + 2]

    stats = ""
    if errs:
        e = sorted(errs)
        stats = (f"konum hatası: medyan {e[len(e)//2]*100:.1f} cm · "
                 f"P95 {e[int(0.95*(len(e)-1))]*100:.1f} cm")

    data = {
        "items": items,
        "racks": rack_boxes(cfg),
        "bounds": [xs[0], xs[1], ys[0], ys[1]],
        "source": args.inventory.name,
        "truth": bool(truth),
        "stats": stats,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(HTML.replace("__DATA__", json.dumps(data, ensure_ascii=False)))

    n_missed = sum(1 for i in items if i["missed"])
    print(f"{len(items) - n_missed} ürün"
          + (f" + {n_missed} kaçan" if n_missed else "")
          + f", {len(data['racks'])} raf gözü")
    if stats:
        print(stats)
    print(f"\n{args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
