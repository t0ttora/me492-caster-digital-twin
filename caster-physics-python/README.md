# Caster fiziği — Python ile anlayarak çalıştır

Bu proje, iki tahrik tekerleği ve bir pasif caster içeren robotun **mevcut M2
mekanik modelini** Python'a taşır. Motor torku verilir; gövde/caster hareketi
hesaplanır. Robotu ideal bir hız komutuyla zorla yürütmez.

**Parametreler sentetiktir.** Sayısal MATLAB eşdeğerliği, gerçek robot
kalibrasyonu veya deneysel doğrulama anlamına gelmez. MATLAB dosyaları korunur.

## İlk çalıştırma

İlk seferde aşağıdaki kurulum adımlarını tamamla. Ardından:

```sh
cd caster-physics-python  # depo kökünden
.venv/bin/python run_experiments.py --group first-lesson
```

Komut iki deneyi çalıştırır: caster başlangıçta **0°** ve **45°**.
Motor torkları aynı kalır. `outputs/` altında:

- `trajectory_comparison.png`: A tahrik aksı orta noktasının izlediği yol.
- `angles_and_energy.png`: gövde yönelimi, göreli caster açısı ve enerji.
- `robot_motion.html`: iki koşuyu yan yana gösteren çevrimdışı animasyon;
  dosyayı tarayıcıda aç. Oynat/duraklat, başa dön ve zaman çubuğu içerir.
- İki `*_states.csv`: grafikleri oluşturan SI veriler.
- `python_run_manifest.json`: kod/parametre/sonuç hash'leri ve çözüm bilgisi.

Animasyon yeni bir fizik simülasyonu değildir; kayıtlı çözüm örneklerini gösterir.
Yörünge **A aks merkezine**, entegre edilen konum **B gövde COM'una** aittir.

## Kodda nereden başlamalıyım?

Önce [robot ve fizik problemi](docs/learning/01-problem-ve-robot.md), ardından
[kaynak dizininin içindeki kod rehberi](src/caster_physics/KODU_OKUMA_REHBERI.md).
En önemli dosya [physics/motion.py](src/caster_physics/physics/motion.py):
`evaluate_motion` bütün fizik zincirini sırayla çağırır.

```text
caster-physics-python/
├── run_experiments.py       # Benim ilk girişim: hangi deney çalışacak?
├── verify_model.py          # MATLAB karşılaştırmasını yeniden üret
├── parameters/             # Çalışma kopyası; SI JSON, isimli parametreler
├── src/caster_physics/      # ASIL KOD: her dosya bir soruya cevap verir
│   ├── state.py            # Robotun konumu, açıları, hızları ve enerji durumu
│   ├── parameters.py       # Parametre adları, birimleri ve geçerlilik kontrolü
│   ├── physics/
│   │   ├── geometry.py     # Tekerlek merkezleri nerede, nasıl hareket ediyor?
│   │   ├── support_loads.py# Ağırlık üç tekerleğe nasıl dağılıyor?
│   │   ├── wheel_slip.py   # Tekerlek ne kadar boyuna/yanal kayıyor?
│   │   ├── contact_forces.py# Kaymayı hangi kuvvet ve momentler karşılıyor?
│   │   ├── inertia.py     # Kütle/atalet hareketleri nasıl birbirine bağlıyor?
│   │   ├── energy.py      # Motor işi nereye gidiyor, enerji dengesi tutuyor mu?
│   │   └── motion.py      # Kuvvetlerden bağlı ivmeleri ve türevleri hesapla
│   ├── simulation/
│   │   ├── experiments.py # Başlangıç koşulları ve 12 tork programı
│   │   ├── rk4.py         # Küçük zaman adımlarında durumu ilerlet
│   │   └── records.py     # Durumu, kuvvetleri ve enerjiyi CSV sütunlarına çevir
│   ├── visualization.py   # Çözümü çiz ve kayıtlı hareketi oynat
│   ├── verification.py    # MATLAB ve fizik kontrollerini hesapla
│   └── result_files.py    # Python sonuçlarını kaynaklara dokunmadan yaz
├── docs/learning/          # Dört kısa Türkçe ders ve cevaplı alıştırmalar
├── tests/                  # Fizik, eşdeğerlik ve kaynak koruma testleri
├── reference/matlab_m2_v1/ # Kayıtlı MATLAB referansının değişmez kopyası
├── verification/          # Başlangıç MATLAB hash envanteri ve test kanıtları
└── outputs/               # SADECE Python'un ürettiği CSV, grafik ve raporlar
```

Kaynakta `x13` veya `p32` taşıma sözleşmesi kullanılmaz. Örneğin
`state.caster_angle_relative_rad`, `torques.left_Nm` ve
`parameters.geometry.caster_trail_m` fiziksel anlamı ve birimi açık adlardır.
Sayısal çözücünün 13 elemanlı vektör sırası yalnız `state.py` içinde tanımlıdır.

## Fizik zinciri

```text
Tork ve başlangıç durumu
  -> geometri / tekerlek merkezlerinin hızları
  -> düşey destek yükleri ve boyuna/yanal kayma
  -> zemin kuvvetleri + yuvarlanma / yatak / bore dirençleri
  -> M @ ivmeler + merkezcil_terimler = genelleştirilmiş_kuvvetler
  -> zamanla hız, konum, caster yönü ve tekerlek hızları
```

Kütle matrisi ters çevrilmez; `numpy.linalg.solve` ile denklem sistemi çözülür.
Kaynaklar: [NumPy solve](https://numpy.org/doc/stable/reference/generated/numpy.linalg.solve.html),
[mevcut M2 türetimi](../models/caster-m2-v1/theory.md).
Kendi modelimizin kaynağı ve sınırlamaları o türetimde açıklanır; yayımlanmış
bir modelin veya gerçek robotun birebir yeniden üretimi iddia edilmez.

## Dört ders

1. [Problem, robot, koordinatlar ve 13 durum](docs/learning/01-problem-ve-robot.md)
2. [Kodu hangi sırayla okuyacağım?](docs/learning/02-kodu-okuma-sirasi.md)
3. [Kuvvetlerden harekete: denklemler ve küçük örnek](docs/learning/03-kuvvetlerden-harekete.md)
4. [İlk deney, grafikler ve hocaya anlatım](docs/learning/04-ilk-deney-ve-sunum.md)

## Diğer deneyler

```sh
.venv/bin/python run_experiments.py --list
.venv/bin/python run_experiments.py --case passive_coast
.venv/bin/python run_experiments.py --group all
```

Varsayılan çıktı isimleri aynı olduğundan aynı deney tekrar çalıştırılınca
**Python'un önceki çıktısı** yenilenir. Bir sonucu tutmak için yeni klasör seç:
`--output outputs/benim-deneyim`. MATLAB/CAD ağaçları ve parametre/referans
klasörleri çıktı hedefi olarak reddedilir. Sembolik bağlantıya yazılmaz.

## MATLAB ile aynı sonucu verdiğini nasıl kontrol edeceğim?

```sh
.venv/bin/python verify_model.py
.venv/bin/python -m pytest -q
```

Doğrulayıcı 12 deneyi yeniden hesaplar. Rapor:
`outputs/verification/verification_report.json`.

- **Üç tam zaman serisi:** `aligned_forward`, `misaligned_forward`,
  `passive_coast`; 44 sütunun tümü karşılaştırılır.
- **12 son durum:** diğer dokuzunun tam MATLAB CSV'si yerelde yoktur;
  kayıtlı manifestteki son x/y/yaw ve caster açısı karşılaştırılır.
- Eşik önceden belirlendi: her değer için `|hata| <= 1e-7 + 1e-6*|referans|`.
  Zaman ve tork sütunlarında daha sıkı `1e-12` mutlak eşik vardır.
- Kaynak koruma: Public depoya eklenen 55 MATLAB kaynak ve kanıt dosyasının SHA-256
  değeri başlangıç envanteriyle karşılaştırılır.
- Fizik: pozitif destek, sürtünme kapasitesi, enerji/momentum, ayna simetrisi
  ve çözüm adımı küçüldüğünde yakınsama ayrıca kontrol edilir.

MATLAB bu Mac'te yeniden çalıştırılmış sayılmaz: referanslar 7 Ekim tarihli
native Windows MATLAB kayıtlarıdır. `reference/` içeriğini değiştirme.

## Parametreyi değiştirmek

Yalnız `parameters/geometry_si.json` ve `parameters/dynamics_si.json` çalışma
kopyalarını düzenle. Örneğin `caster_trail_m` pivot ile caster aksı arasındaki
mesafedir. Kaynak MATLAB dosyaları ve `reference/` sabit kalır.
Parametre değiştirince aynı MATLAB sonucunu bekleme; karşılaştırma o farkı
bulmak için vardır. İlk öğrenme deneylerini varsayılan değerlerle bitir.

## Başka bilgisayarda kurulum

Python **3.10 veya üstü** gerekir. Bu klasörü bütünüyle kopyala; `.venv`
başka bilgisayara taşınabilir değildir, yeniden oluştur:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
```

Windows'ta aynı komutlarda `.venv/bin/python` yerine `.venv\Scripts\python.exe`
kullan. Geliştirme için alternatif: `python -m pip install -e ".[test]"`.
Bu proje yerel/editable kullanım içindir; parametre ve referans klasörlerini
çıkartıp yalnız paketi wheel olarak kurmak desteklenmez.

## Modelin sınırı

Düz zemin, sabit yükseklik/roll/pitch, quasi-static düşey yükler, yumuşak kayma
sürtünmesi, toplulaştırılmış caster kütlesi ve tekerlek aksı tork girdisi.
Tam stiction, esnek lastik shimmy, darbe/devrilme, motor elektrik devresi veya
gerçek robot hız servosu içermez. Caster teğetsel etkilerini kapatmak üçüncü
tekerleği/kütleyi kaldırmaz. Ölçülmüş robot davranışı için veri ve kalibrasyon gerekir.
