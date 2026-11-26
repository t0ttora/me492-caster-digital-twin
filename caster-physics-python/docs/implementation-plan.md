# Python fizik modeli — onaylanmış kapsam ve uygulama planı

Tarih: 19 Ekim 2026. Kullanıcı Python geçişini, MATLAB'ın korunmasını,
aynı sonuçların test edilmesini ve anlaşılır kod dizinini açıkça onayladı.

## Tasarım

Mevcut M2'nin düzlemsel denklemleri korunur. NumPy matris çözümü ve açık RK4
zaman adımları kullanılır; yeni fizik motoru, Simscape, AI veya web arayüzü yoktur.
Kaynak, `src/caster_physics/` altında fiziksel görevine göre adlandırılır.
Durum ve parametre alanları isimlidir; sayısal vektör sırası yalnız state.py'da
tanımlıdır. Açıklamalar Türkçe, fonksiyon adları fiziksel anlamı belirten İngilizcedir.

Ana klasör: `caster-physics-python`. Ana MATLAB klasörüne yazma yapılmaz.
Üst Git deposu ilk commit'i olmayan, tüm dosyaları untracked bir depodur;
HEAD olmadığı için bu depodan worktree üretilemez. Kullanıcı dosyalarını commit
ederek bunu değiştirmek yerine yeni, ayrı klasörde çalışılır.

## Kabul koşulları

- MATLAB kaynaklarının başlangıç/son SHA-256 değerleri aynı.
- 13 durum ve SI parametreleri korunur; A aks merkezi ve B COM ayrımı açık.
- Sabit ode4 eşdeğeri RK4 ve discrete/ZOH komut zamanlaması korunur.
- Üç MATLAB CSV'sinin tüm sütunları zaman boyunca toleransla karşılaştırılır.
- 12 senaryonun kayıtlı son konum ve caster açıları karşılaştırılır.
- Pozitif destek yükleri, enerji, sürtünme sınırı, simetri, yakınsama test edilir.
- Tek komutlu başlangıç, açıklamalı iki deney, okunabilir grafik ve animasyon.
- Sentetik sayısal eşdeğerlik gerçek robot kalibrasyonu diye sunulmaz.

## Uygulama adımları

1. `reference/matlab_m2_v1/`: değişmez JSON/CSV referanslarını kopyala;
   `verification/matlab_sources_before.json`: kaynak hash envanteri (düşük risk).
2. `tests/test_reference_parity.py`, `tests/test_physics.py`: önce RED;
   karşılaştırma eşikleri uygulama öncesi belirlenir (orta risk).
3. `src/caster_physics/state.py`, `parameters.py`: isimli SI sözleşmesi (düşük).
4. `physics/geometry.py`, `support_loads.py`, `wheel_slip.py`,
   `contact_forces.py`, `inertia.py`, `energy.py`, `motion.py`:
   tek fiziksel sorumluluklu denklemler; referansın varsayımları korunur (orta).
5. `simulation/experiments.py`, `rk4.py`, `records.py`: 12 deney, ayrık
   komutların RK4 alt adımları boyunca tutulması ve CSV tanıları (yüksek).
6. `run_experiments.py`, `verify_model.py`: CLI, tekrar üretim, JSON kanıtı (orta).
7. `visualization.py`, `docs/`: iki deneyin grafikleri, HTML/SVG animasyonu,
   kod okuma sırası ve mekanik açıklama; robot doğrulama sınırı (düşük).
8. Bağımsız kod/Python incelemesi, gerekli düzeltmeler; taze pytest, kapsam,
   tüm deneyler, referans ve kaynak koruma denetimi. Sonuç raporu korunur;
   geçici hata çıktıları temizlenir.

## Test stratejisi

Fizik testleri formüllerin kopyası değil, bağımsız kuvvet/moment/enerji
beklentileridir. Entegrasyon testleri MATLAB kayıtlarına karşıdır.
CSV kıyas eşiği: her sütun için `abs(error) <= 1e-7 + 1e-6*abs(reference)`;
zaman ve komutlar ayrıca daha sıkı test edilir. Fiziksel durumlar ve türetilen
kuvvet/enerji sütunları ayrı raporlanır. Son durum eşiği aynı koşuldur.
Eşik başarısızlığı sessiz gevşetilmez; nedeni incelenir ve varsa raporlanır.

## İlerleme

- [x] Kullanıcı kapsamı ve Python geçişini onayladı.
- [x] Kaynak ve ZOH/ode4 ayarları okundu.
- [x] RED -> GREEN fizik ve MATLAB karşılaştırma testleri.
- [x] 12 senaryo ve grafikler üretildi.
- [x] Açıklamalı öğrenme akışı tamamlandı.
- [x] Bağımsız kod/Python incelemesi; çıktı bağlantısı ve A/B anlatım bulguları düzeltildi.
- [x] Son doğrulama: 54 test, 3 tam CSV (44 sütun), 12 son durum, 78 fizik kontrolü.
- [x] 258 mevcut kaynak dosya değişmedi; referans kopyaları hash ile doğrulandı.

Son kanıtlar: [doğrulama özeti](../verification/README.md).
HTML tarayıcı etkileşim kontrolü dosya-protokol erişimi nedeniyle yapılmadı;
HTML veri/geometri testleri ve gömülü JavaScript sözdizimi kontrolü geçti.
Grafikler gerçek Python çıktılarından görsel olarak incelendi.
