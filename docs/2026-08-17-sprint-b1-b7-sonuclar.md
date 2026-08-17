# WareDrone Sprint — B1–B7 sonuçları (ibrahimcobankose, Perception & Inventory)

Tarih: 2026-08-17. Üç tam tur koşusunun sonuçları. Uçuş rotası `tam_tur_geo`
(4 raf yüzü × 3 seviye, 24 waypoint, GPS yok).

## Depo ölçeği (ekip liderinin metnindeki 54 rakamı bize ait değil)

Sprint metnindeki "52/54 QR" başka bir kurulumdan; bizim dünyamızda
**216 kutu QR'ı + 216 barkod (Code128) + 84 raf AprilTag + 16 zemin AprilTag**
var. Payda her yerde 216.

## Üç koşuluk KPI

| | Koşu 1 | Koşu 2 | Koşu 3 | Hedef |
|---|---|---|---|---|
| QR tespit | 216/216 | 216/216 | 216/216 | ≥%98 |
| Raf kapsaması (geçiş) | 12/12 | 12/12 | 12/12 | tümü |
| Envanter kaydı | 216 | 213 | 215 | — |
| Envanter doğruluğu | %100 | %100 | %99.5 | ≥%95 |
| Konum hatası (medyan) | 4.2 cm | 3.8 cm | 3.9 cm | — |
| Barkod bağlama doğruluğu | — | %100 | %100 (592/592) | — |
| Barkod kapsaması | — | %63 | %61 | — |
| Algılama FPS | 6.8 | 6.8 | 7.3 | **≥10 (tutmadı)** |

Güven eşiği uygulanınca (koşu 3, `--min-confidence 0.3`): 189 kayıt,
yanlış göz 0, P95 11.1 cm, maks 20.3 cm, doğruluk %100.

## Durum

**Tamamlandı:** B1 (QR), B4 (envanter veri kümesi), B5 (ground truth doğrulama),
B6 (kapsama raporu), B7 (3B görselleştirme).
**Kısmen:** B2 (barkod — hat çalışıyor, kapsama %61-63), B3 (FPS — 6.8→7.3,
hedef 10 tutmadı).

## Bilinen problemler

1. **FPS 7.3 (hedef 10).** Kamera 20 Hz yayınlıyor; kare boşlukları 0.05 s'nin
   katlarında dağılıyor, %38.5'inde ardışık iki kareyi de işliyoruz. Kalan tek
   darboğaz `pyzbar.decode`'un kendisi (67 ms/kare). Çözüm: 2-3 paralel çözme
   thread'i (zbar'ın C çağrısı ctypes üzerinden GIL bırakıyor). Çözünürlük
   düşürmek seçenek değil — 1080p'de okunabilirlik zaten 4.42 px/modül.
   Kapsama üç koşuda da %100 olduğu için bu marj meselesi, arıza değil.
2. **Kaçan kutular algılama değil LOKALİZASYON kaynaklı.** QR üç koşuda da
   216/216 okundu; envantere giremeyen kutuların (koşu 2: 3, koşu 3: 1) 17-32
   okuması VAR ama hepsi `ev_feed`'de >0.30 s poz boşluğuna denk geldi. Üçü de
   L1 (en alçak seviye, EV'nin en zayıf olduğu yer). `--max-dt 0.5` ikisini,
   `1.0` üçünü geri getiriyor (doğruluk pratikte değişmiyor) ama varsayılan
   temkinli tarafta bırakıldı; asıl çözüm lokalizasyon tarafında.
3. **Barkod kapsaması %61-63.** Beklenen: okunabilirlik bütçesi barkodu sınırda
   gösteriyor (1.5 m'de 3.92 px/modül, eşik 3.0). Bağlanan her barkod doğru
   (%100), sorun okuma oranında.
4. **Hatalı kayıtların hepsi düşük güvenli.** Koşu 3'teki tek yanlış göz
   ataması conf 0.09 / spread 0.95 m ile geldi. Güven skoru hatayı öngörüyor;
   düşük güvenli kayıtlar gerçek bir WMS'te insana yönlendirilir.

## Sonraki geliştirme önerileri

- Paralel çözme ile FPS'i 10+'a çıkarmak.
- EV poz boşluklarını kapatmak (lokalizasyon tarafı) — kaçan kutuları sıfırlar.
- Barkodu büyütmek ya da kameraya pitch vermek (barkod kapsaması).
- Uçuş sırasında canlı envanter (şu an poz eşlemesi uçuş sonrası offline).

## Üretilen dosyalar (her koşuda `out/` altında yeniden üretilir)

`scans/scans.json`, `scans/readings.jsonl`, `scans/frames.csv`,
`inventory.json`, `inventory.csv`, `validation_report.json`,
`coverage_report.json`, `coverage.html`, `inventory_3d.html`.

## Çalıştırma

Uçuş sırasında (ayrı terminaller): `./scripts/run_sim.sh` ·
`scripts/apriltag_localize.py` · `scripts/scan_boxes.py --no-bridge --with-barcode` ·
`bash scripts/flight_logged.sh tam_tur_geo --routes config/route_gen.yaml`

Uçuş sonrası, bu sırayla:

```
.venv/bin/python tools/build_inventory.py --truth
.venv/bin/python tools/validate_inventory.py --list-worst 5
.venv/bin/python tools/coverage_report.py --html out/coverage.html
.venv/bin/python tools/view_inventory.py --truth
```

`view_inventory` ham okumaları değil `inventory.json`'ı okur; `build_inventory`
koşmazsa hata vermeden ESKİ uçuşu gösterir.
