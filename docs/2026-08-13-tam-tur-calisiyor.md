# Kilometre taşı: tam tur temiz uçtu (2026-08-13)

Bu not, projenin ilk **uçtan uca temiz tam turuna** ait durumu ve oraya nasıl
gelindiğini kaydeder. Bir şey bozulursa geri dönülecek nokta burasıdır.

## Sonuç

`tam_tur_geo` rotası, 791 saniyede **24/24 nokta**, en fazla sapma **0.29 m**
(sınır 1.0, raf yüzü 1.5 m), ardından iniş ve disarm. İki koridor × iki raf yüzü
× üç seviye tek uçuşta tarandı.

Gerçek konum (Gazebo ground truth, `out/state_log.csv`) ile doğrulama:

| Ölçüt | Bozuk koşu (öncesi) | Bu koşu |
|---|---|---|
| Max yatay hız | 9.91 m/s | **0.85 m/s** |
| L1 max hız | 2.4+ m/s | **0.54 m/s** |
| Koridor/köşe dışına taşma | x=49 m'ye kaçış | **0 örnek** |
| Tamamlanan nokta | 5/24 | **24/24** |

Lokalizasyon: **5275 EV fix** (3383 floor + 1892 rack), kesintisiz.

## Kök neden: dokusuz zemin

Aylardır "L1 kararsızlığı / anlamsız hızlanma" diye görünen şeyin sebebi tek bir
şeydi: **depo zemininin dokusuz düz gri olması.**

Optik akış sensörü zemindeki görsel özellikleri izleyerek çalışır. Düz renk bir
zeminde izlenecek hiçbir şey yok — gz akış eklentisi sürekli
`Number of good matches: 2, desired: 20` diyordu. Zincir şöyle işliyordu:

    dokusuz zemin -> akış özellik bulamıyor -> bozuk hız üretiyor
    -> EKF durumu kayıyor -> kontrolcü "geri kaldım" sanıp tam gaz -> runaway

Bu, düşük irtifada (L1, ~0.42 m) en şiddetliydi; L2/L3 zaten temizdi.

### Teşhisi mümkün kılan şey

`out/logs/ev_feed.csv` (her EV fix'i duvar saatiyle) + `state_log.csv`'ye eklenen
`wall_ms` sütunu hizalanınca soru kesin cevaplandı: **kaçış anında raf-tag görüşü
kesintisiz ve cm-doğru besliyordu** (4–25 cm hata). Yani lokalizasyon sağlamdı,
bozuk olan akıştı. Araç nerede olduğunu biliyordu; sorun ölçü/geometri eksikliği
değildi.

### Neden EKF ayarıyla çözülemedi

Sensörün kendisi bozuk olduğu için EKF tarafındaki her çare ya yetersiz kaldı ya
yeni bir şeyi kırdı:

- `EKF2_OF_CTRL 0` (akışı kapat) → **arm kırıldı**:
  `Preflight Fail: heading estimate not stable`.
- Görüş hızı beslemek (`VISION_SPEED_ESTIMATE` + `EKF2_EV_CTRL 13`) → yetmedi,
  çünkü sorun yön/hız gözlenebilirliğiydi.
- Akışı zayıflatmak (`OF_N_MIN/N_MAX/GATE`) → arm yine kırıldı.

**Neden akış vazgeçilmez:** manyetometre ve GPS yok; ayrıca araç yerdeyken alt
kamera zeminin altında kalıyor (base_link −0.013, kamera −0.08 → z ≈ −0.09), yani
spawn'da zemin tag'i görülemiyor. Dolayısıyla **yerdeki tek yatay yardım optik
akıştır** (`EKF2_OF_QMIN_GND=0` yerde düşük kaliteyi kabul eder). Akışı kısmak
kalkışı imkânsız kılıyor.

## Bu noktadaki yapılandırma

- **EKF parametreleri: varsayılan.** Deneme amaçlı hiçbir override yok
  (`parameters.bson` sıfırlandı). Airframe `px4/airframes/4022_gz_warehouse_scout`
  param bazında sağlıklı sürümle aynı.
- **Zemin dokulu:** `tools/gen_world.py` → `floor_texture()` + `floor_mesh_obj()`,
  `FLOOR_TILE_M = 0.5`, tekrarlı UV. Kutu/tag düzeni ve `ground_truth` değişmedi.
- **Rota geometriden üretiliyor:** `scripts/gen_route.py` →
  `config/route_gen.yaml` (`tam_tur_geo`, `koridor1_geo`).

Çalıştırma:

    ./scripts/run_sim.sh
    source /opt/ros/jazzy/setup.bash && .venv/bin/python scripts/apriltag_localize.py
    .venv/bin/python scripts/log_state.py
    bash scripts/flight_logged.sh tam_tur_geo --routes config/route_gen.yaml

## Tuzaklar (tekrar yaşanmasın)

1. **Bayat Gazebo sunucusu.** PX4 kapanınca `gz sim` server ayakta kalabiliyor;
   PX4 yeniden başlayınca O BAYAT sunucuya bağlanıyor. Israrla giden
   `vehicle_imu timestamp error` ve lockstep tuhaflıklarının sebebi budur.
   Kapatırken doğrula: `pgrep -af "px4|gz sim"`.
2. **`pxh>` konsolu akış spam'iyle boğulur.** `param set/show` çıktısı görünmez.
   Bunun yerine `scripts/set_param.py` (MAVLink'ten set + geri okuyup doğrular).
3. **Param'ları tek tek geri almak yetmeyebilir.** Toptan dönüş için
   `rootfs/parameters.bson`'ı yedekleyip sil; PX4 açılışta airframe
   varsayılanlarından yeniden üretir.
4. **Semptomun katmanını doğrula.** Runaway aylarca rota/EKF/kontrol sorunu
   sanıldı; ölçüm (EV log + ground truth hizalaması) sensör katmanını gösterdi.
   Önce "hangi veri bozuk" sorusunu cevapla, sonra ayar yap.

## Açık işler

- `scan_boxes` entegrasyonu ve kapsama raporu bu tam turla henüz koşulmadı
  (`tools/coverage_report.py`).
- Dikey kestirimde ~0.375 m sabit sapma duruyor (bilinçli; yükseklik referansı
  mesafe ölçer, `EKF2_EV_CTRL` dikey biti kapalı).
- `check_readability.py` / `probe_labels.py` hâlâ QR varsayıyor, AprilTag'e
  güncellenmedi.
