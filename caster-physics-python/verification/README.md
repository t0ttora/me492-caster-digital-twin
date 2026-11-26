# Gerçekte ne çalıştırıldı? — 19 Ekim 2026

Python 3.12.14, NumPy 2.5.4; bağımlılıklar `../requirements-lock.txt` içinde.

## Yeniden üretim

Proje kökünde:

```sh
.venv/bin/python run_experiments.py --group first-lesson
.venv/bin/python verify_model.py
.venv/bin/python -m pytest --cov=caster_physics --cov-report=term-missing --cov-report=json:verification/coverage.json --junitxml=verification/tests.xml -q
```

| Kontrol | Bu oturumdaki sonuç |
|---|---|
| İlk öğrenme komutu | 0° ve 45° deneyleri çalıştı; her biri 401 kayıt |
| Testler | **54 geçti**, 0 başarısız; 67,11 saniye |
| Paket satır kapsamı | **%97**; bu oran tek başına fizik doğruluğu değildir |
| Tam MATLAB zaman serisi | **3/3**; 401, 401 ve 501 satır, her birinde 44 sütun |
| Kayıtlı MATLAB son durumları | **12/12**; x/y/yaw ve göreli caster açısı |
| Python koşularında fizik kontrolleri | **78/78** |
| Mevcut kaynak koruma | **258/258** SHA-256 aynı; kayıp/değişen dosya yok |
| Kopyalanan referans bütünlüğü | **7/7** veri/metadata dosyası hash ile aynı |
| Doküman bağlantıları | **38/38** yerel bağlantı mevcut |
| Çıktı dosyası symlink koruması | CSV/JSON/PNG/HTML dört regresyon testi geçti |
| Bağımsız inceleme | Fizik/Python incelemesi ve düzeltilen bulguların tekrar incelemesi tamam |

Test çıktısının kalıcı kanıtı: [tests.xml](tests.xml) ve [coverage.json](coverage.json).
Sayısal karşılaştırmanın ayrıntısı:
[verification_report.json](../outputs/verification/verification_report.json).
Çalıştırılan Python kaynak/parametre/CSV hash'leri:
[python_run_manifest.json](../outputs/verification/python_run_manifest.json).

## Farklar ne kadar?

Eşik implementasyondan önce `1e-7 + 1e-6*abs(MATLAB)` olarak belirlendi;
başarısız karşılaştırmayı geçirmek için değiştirilmedi. Zaman ve motor torklarında
ayrıca `1e-12` mutlak eşik kullanıldı.

| Tam zaman serisi | En büyük x farkı [m] | En büyük yaw farkı [rad] |
|---|---:|---:|
| aligned_forward | 6,11e-16 | 0 |
| misaligned_forward | 6,11e-16 | 5,11e-15 |
| passive_coast | 6,66e-16 | 7,77e-16 |

12 son durumda karşılaştırılan dört büyüklükte en büyük mutlak fark
2,22e-15'tir (karışık m/rad sütunları; ortak fiziksel birim değildir).
Sütunların her birinin kendi birimindeki mutlak ve normalize hatası JSON raporundadır.

## Kanıtın sınırları

- Python bu oturumda gerçekten çalıştırıldı. MATLAB bu oturumda yeniden
  çalıştırılmadı; karşılaştırma 7 Ekim native Windows MATLAB kayıtlarınadır.
- Diğer dokuz deneyde yerelde tam MATLAB CSV'si yoktur. Bunlar için
  **tam zaman serisi eşdeğerliği** iddiası yok; son durum karşılaştırması vardır.
- Parametreler sentetiktir. Gerçek robot ölçümü, sürüşü veya kalibrasyonu yoktur.
- PNG grafikler Python sonuçlarından üretildi ve görsel olarak incelendi.
- `robot_motion.html` üretildi; kayıtlı örnek/geometri ve çevrimdışı içerik
  testleri, gömülü JS sözdizimi kontrolü geçti. Tarayıcı render/etkileşim testi
  dosya protokolü erişim sınırı nedeniyle yapılmadı.
- Üst Git deposu HEAD içermediği ve orijinal dosyalar untracked olduğu için
  kaynakları commit ederek değiştirmedik; uygulama yeni klasörde tutuldu.
  Commit, push, yayın veya robot üzerinde yürütme yapılmadı.
