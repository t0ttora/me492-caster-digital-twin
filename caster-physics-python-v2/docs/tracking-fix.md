# V2.1 takip kontrolü düzeltmesi

13 Kasım 2026. V1 kaynakları, parametreleri ve MATLAB referansları değişmedi.
Eski V2 kaynakları `evidence/v2-before-tracking-fix-2026-11-13.tar.gz` içinde,
eski 40 koşu `outputs/validation-2026-11-13` altında korundu.

## Neden yalnız üç koşu bitiyordu

Eski dış kontrolün yanal düzeltmesi `3*v_ref*e_y` idi. Profil bitince
`v_ref=0` olduğundan yanal düzeltme kayboluyordu. Hedefle aynı boyuna konum
ve yaw'da fakat 3 cm yanda duran robot için iki kontrol komutu da sıfırdı.
Bu, duruş hedefinde yanal hatalı bir denge noktası oluşturuyordu.

Tekerleklerde yalnız P kontrol vardı. Sabit yük altında `J*rate_dot=tau-d`
ve `tau=Kp*error` modelinde denge hatası `d/Kp` olur. M2 yükleri değişkendir;
bu alt sistem hesabı PI ihtiyacını açıklayan basit karşı örnektir.

Bir saniyelik son gözlem de bazı koşular için kısaydı. Fakat eski kontrolü
yüksek dirençli zemindeki dönüşün ardından sekiz saniye daha çalıştırmak
konum hatasını gidermedi: 36.80 mm hata, −35.80 mm yanal bileşen kaldı.
Dolayısıyla yalnız zaman aşımını uzatmak çözüm değildi.

## Değişen yerler

- `twin.py`: `TrackingController`, geometrik takip hatasına göre boyuna ve yaw
  komutlarını hesaplıyor. Tekerlek PI servosu koşullu integrasyonla ±0.5 Nm
  doygunluğunu artıran integral birikimini engelliyor.
- Profil sonunda konuma ileri/geri polar yaklaşım, sonra hedef yaw hizalaması
  yapılıyor. 6 mm'de hizalamaya geçiliyor; sürüklenme 12 mm'yi aşarsa konum
  yaklaşımına dönülüyor. Tüm modlarda aynı hız/yaw/tork sınırları geçerli.
- `completion_time`: 2 cm, 0.05 rad, 0.02 m/s ve 0.05 rad/s eşiklerini koruyor.
  Hız ölçümü toplam düzlemsel hıza sıkılaştırıldı; kalış 0.25 s zaman damgasıyla
  denetleniyor. Tamamlanma zamanı, doğrulanmış kalışın başlangıcıdır.
- `run_case`: açıkça belirtilmiş 8 s terminal gözlem kullanıyor. Eski 1 s
  pencerede bitiş ve aynı zaman penceresindeki hata ayrıca raporlanıyor.
- `run_study.py`: sürüm 2.1, kontrol sabitleri ve tüm koşuların bitiş denetimi
  manifestte bulunuyor. Kontrol komutları, mod kodu ve integral torkları CSV'de.
- `evaluate_recording.py`: değişmedi; gerçek tork kaydı tekrarı, sentetik
  kontrolcüden bağımsız aynı mekanik çekirdeği kullanıyor.

## Çalıştırılan sonuçlar

| Karşılaştırma | Önce | V2.1 |
| --- | ---: | ---: |
| Nominal matris, profil sonu +1 s | 3/16 | 8/16 |
| Nominal matris, profil sonu +8 s | Eski 16 koşunun tümü bu ufukta çalıştırılmadı | 16/16 |
| Duyarlılık dahil, profil sonu +8 s | Aynı ufukta eski tam matris yok | 40/40 |
| Aynı eski pencerede konum RMSE iyileşen nominal koşu | Referans | 16/16 |

Yeni 40 koşuda en uzun toparlanma 3.28 s; 8 s bir üst gözlem sınırıdır.
Nominal koşuların 8 s gözlem sonundaki en büyük konum hatası 7.08 mm,
en büyük yaw hatası yaklaşık 0.000746 rad'dır. Profilin kendi süresi değişmedi.
Terminal düzeltmeler ek hareket içerir; tamamlanmanın iyileşmesi tek başına
jerk veya yol uzunluğunun her koşuda iyileştiğini göstermez.

Kanıtlar:

- `outputs/tracking-fix-v2.1-2026-11-13/study.json`: 40/40 tamamlandı;
  enerji, pasif temas ve tork sınırı kontrolleri geçti.
- `evidence/tracking-fix-comparison.json`: 16 koşunun aynı pencere karşılaştırması.
- `evidence/tracking-before-diagnostics.json`: önceki bitiş kapıları ve hata bileşenleri.
- `evidence/old-controller-extended-horizon.json`: eski kontrolde yalnız süre uzatma deneyi.
- `evidence/tracking-regression-red.xml`: değişiklik öncesi beş başarısız regresyon.
- `evidence/v2.1-tests.xml`: 31 test geçti. V1 yeniden: 54 test geçti.
- `evidence/closed-loop-step-refinement-v2.1.json`: yüksek dirençli standart
  dönüşte adım 0.5 ms'den 0.25 ms'ye indi; aynı 4.45 s bitiş zamanı, en büyük
  yaw farkı yaklaşık 5.6e-9 rad, caster açısı farkı yaklaşık 8.4e-9 rad.

## Matematiksel kanıtın sınırı

PDF, M'nin pozitif tanımlılığını ve seçilen mekanik modelin kuvvet/enerji
denklemlerini türetiyor. Takip Lyapunov özdeşliği, polar terminal yaklaşım,
son açı hizalaması ve sabit yükte PI yük reddi kendi belirtilen ideal
varsayımları altında ispatlanıyor. Örneklemeli skaler PI için Jury koşulları
da veriliyor. Tam, kaymalı, doygun, örneklemeli ve hibrit M2 kapalı çevriminin
küresel yakınsaması ispatlanmış sayılmaz. 40 koşu bu daha geniş iddianın
yerine geçmez; gerçek robot üzerinde doğrulama yapılmadı.
