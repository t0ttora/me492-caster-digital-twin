# ME492 — kendi Simulink modelimiz, M0

Bu paket, iki bağımsız tahrik tekerleği ve tek pasif swivel caster içeren temsili bir platformun **çalışan kinematik başlangıç modelidir**. Başka bir çalışmanın SLX dosyası kopyalanmadı; model temel Simulink bloklarıyla programatik olarak oluşturuldu. Bütün boyutlar sentetiktir; ARCHE robotunun ölçüleri veya saha verileri değildir.

M0, verilen doğrusal ve açısal hızı ideal olarak gerçekleştiren gövdenin hareketini ve buna uyum sağlayan caster açısını hesaplar. Caster kuvvetleri gövdeye geri beslenmez. Bu nedenle model, caster kaynaklı yaw sapmasını, sürtünmeyi, motor yükünü veya shimmy titreşimini doğrulamaz. Simscape ve Simscape Multibody çalışır durumda olsa da bu ilk SLX yalnızca Simulink kullanır.

## Çalıştırma

MATLAB Current Folder'ı bu README'nin bulunduğu `caster_model_v0` klasörüne getir. Command Window'da:

```matlab
run_me492_demo;
verify_me492_model;
open_system(fullfile('models','Me492CasterM0.slx'));
```

İlk komut altı senaryoyu çalıştırır, `commands/`, `results/`, `plots/` ve `run_manifest.json` dosyasını üretir. İkinci komut CSV sonuçlarını bağımsız analitik beklentilerle karşılaştırır ve `verification_evidence.json` yazar. Üçüncü komut kendi modelimizi açar. SLX içinde varsayılan olarak dört saniyelik, 0,2 m/s düz sürüş kayıtlıdır; doğrudan **Run** düğmesiyle de çalışır.

`parameters_si.json` ortak sayısal kaynaktır. Parametre değiştirince `run_me492_demo` bunları model çalışma alanına yeniden yükler. Doğrudan SLX açıp Run'a basmak, SLX içine ilk oluşturulduğunda kaydedilmiş değerleri kullanır. Kaydedilmiş modeli yeniden oluşturmak istersen mevcut SLX'i önce farklı bir adla koru; `build_me492_model` mevcut modelin üzerine yazmayı reddeder. Runner, sonuç CSV ve PNG dosyalarını yeniden üretir; elle değiştirdiğin sonuçları önce farklı klasörde sakla. Modeli açıkken elle düzenliyorsan önce kaydet ve kapat; runner kaydedilmemiş model düzenlemesi varsa çalışmayı reddeder ve deney sonrasında kendi çalışma kopyasını kaydetmeden kapatır.

Doğrulayıcı bu paketin özgün parametre ve senaryolarını sınar. Parametreleri değiştirmen, eski sayısal beklentilerin de değişmesini gerektirir; başarısız bir kontrol o durumda tek başına model hatası sayılmaz.

## İlk öğretici inceleme: sinyalden harekete

1. Üst seviyede `MotionCommands` bloğunu bul. CSV'nin ilk sütunu zaman, diğer ikisi gövde doğrusal hızı `v` ve açısal hızı `omega`dır. `From Workspace` bunları zamanla modele verir; `SplitCommands` iki sinyale ayırır. CSV örnekleme aralığı 0,02 s; örnekler arasında komut sabit tutulur.
2. `IdealBodyPose` içine çift tıkla. `Yaw` bir Integrator'dır: açısal hızı zamana göre biriktirerek yönelim açısını hesaplar. `CosYaw` ve `SinYaw`, doğrusal hızın dünya x/y bileşenlerini belirler. `X` ve `Y` integratörleri bunları konuma dönüştürür. İntegratör çıkışı bir **durum**, girişiyse o durumun değişim hızıdır.
3. `PassiveCasterTracking` içine gir. `Phi` caster'ın gövdeye göre açısını tutar. `SinPhi` ve `CosPhi`, pivot hızını caster'a göre bileşenlerine ayırır. Yanal pivot hızının trail'e bölünmesi mutlak caster dönüş hızını verir. Sonra gövdenin açısal hızı çıkarılır; böylece gövdeye göre açı türevi elde edilir.
4. `DriveWheelRates` gövde komutunu sol/sağ tekerlek açısal hızına çevirir. Bu bir motor modeli değildir; motor gecikmesi, tork limiti veya kayma içermez.
5. `StateScope` dört farklı birimi aynı ekranda gösterebilir; fiziksel karşılaştırma için birimleri belirtilmiş PNG grafiklerini kullan. `StateRecord` ve `RateRecord`, CSV üretimi için simülasyon çıktısını kaydeder.

İlk gözlem: `straight_misaligned` senaryosunda caster 45° ile başlar ve ileri sürüş sırasında hizalanır. Buna rağmen gövde yolu, `straight_aligned` ile birebir aynıdır. Bu, M0'ın bilinçli sınırıdır; gerçek robotta aynı durumun yaw sapması üretip üretmediğini daha sonra ölçmemiz gerekir.

## Denklem ve açı tanımı

Gövde merkezi, tahrik aksı orta noktasının zemine izdüşümüdür. +X ileri, +Y sol, +Z yukarıdır. Pozitif yaw yukarıdan bakıldığında saat yönünün tersidir. `theta` gövdenin dünya yönelimi, `phi` caster'ın gövdeye göre yuvarlanma yönü, `psi = theta + phi` caster'ın dünya yönelimidir. Açılar CSV'de radyan ve süreklidir; grafiklerde dereceye çevrilir.

Pivotun gövde koordinatları `a, b`, pozitif trail `l`, tahrik yarıçapı `r`, iz genişliği `B` olsun:

```text
x_dot = v cos(theta)
y_dot = v sin(theta)
theta_dot = omega

hinge_vx = v - omega b
hinge_vy = omega a
phi_dot = (-hinge_vx sin(phi) + hinge_vy cos(phi)) / l - omega

left_wheel_rate  = (v - omega B/2) / r
right_wheel_rate = (v + omega B/2) / r
```

Caster tekerleği pivotun `l` kadar gerisindedir. Kaymasız yuvarlanma kısıtından türetilen caster denklemi, gövdenin hızını dışarıdan verilmiş kabul eder. Kütle, atalet ve temas kuvveti hesaplanmaz. Dünya caster tekerlek merkezi: `C = H - l [cos(psi), sin(psi)]`; `H` dünya pivot konumudur.

## Senaryolar ve yorum

| Senaryo | Girdi ve ilk caster açısı | Beklenen gözlem |
|---|---|---|
| straight_aligned | 0,2 m/s, 4 s; 0° | x = 0,8 m; caster açısı sıfır |
| straight_misaligned | aynı komut; 45° | caster hizalanır; gövde yolu aynıdır |
| reversal_perturbed | 2 s ileri, 6 s geri; 1° | net x = −0,8 m; caster yaklaşık 180°'ye döner |
| reversal_exact_aligned | aynı komut; tam 0° | ideal denklemde kararsız denge korunur |
| turn_in_place | v = 0, omega = 0,5 rad/s; 4 s | x/y sabit; yaw = 2 rad |
| curved_turn | v = 0,2, omega = 0,4; 5 s | yarıçap 0,5 m; yaw = 2 rad |

Tam hizalanmış ters sürüşte otomatik 180° dönüş eklenmedi. İdeal denklemin bu özel dengesi gerçek robotun kaçınılmaz küçük bozukluklarını temsil etmez. Ayrı 1° başlangıç senaryosu bu farkı görünür kılar; rastgele veya gizli bir açı sıçraması kullanılmaz.

Simülasyon `ode4`, sabit 0,001 s adımla çalışır. 2026-10-07 tarihinde bu ortamda MATLAB R2026b, Simulink, Simscape ve Simscape Multibody yükleme/lisans kontrolü ve küçük bir integratör deneyi başarıyla çalıştı; ayrıntı `capability_evidence.json` dosyasındadır. Sayısal model doğrulaması ile robot üzerinde fiziksel geçerleme farklı işlerdir.

## Sonraki fiziksel modeller

**M1:** Ölçülen tahrik gecikmesi, hız cevabı ve gerekirse yük bağımlılığı; caster yük geri beslemesi olmadan gövde davranışı. Bu, caster modelinin katkısını yanlışlıkla motor modeline atfetmemeyi sağlar.

**M2:** Verinin ayırt edebildiği caster swivel direnci ve temas dinamiği, gövdeye kuvvet/moment geri beslemesi. M1–M2 karşılaştırması ayrılmış test koşullarında yapılmalı. Her sürtünme, esneklik veya shimmy etkisini ilk aşamada eklemek yerine ölçülebilen etkiden başlanmalı.

Bu paket Gazebo'yu çalıştırmaz; ortak JSON, komut CSV'leri ve başlangıç durumlarıyla sıralı deney tekrarının MATLAB tarafını hazırlar. CAD ve Gazebo kurulumunun doğru kinematik eksenleri kullanması gerekir; dosya aktarımı tek başına iki simülatörün eşdeğerliğini kanıtlamaz. Ortak sözleşme için `schema.md` dosyasına bak.

Öğretici adımlar kişisel çalışma rehberidir; resmî araştırma roadmap'ine ayrı öğrenme iş paketi eklenmez. 9 Ekim Cuma görüşmesine hazırlık için bu M0 demo kullanılabilir; sentetik parametre ve fiziksel geçerleme sınırı sunumda açıkça belirtilmeli.

## Kaynaklar

Denklemler bu paketin geometri ve kaymasız yuvarlanma varsayımından türetildi. Literatür araştırması üst klasördeki `caster_matlab_simulink_arastirma.md` dosyasında yer alır; referanslardaki başarı bu modelin fiziksel doğrulaması sayılmaz. Programatik model ve veri girişi için MathWorks'ün [add_block](https://www.mathworks.com/help/simulink/slref/add_block.html), [From Workspace](https://www.mathworks.com/help/simulink/slref/fromworkspace.html), [ModelWorkspace](https://www.mathworks.com/help/simulink/slref/simulink.modelworkspace.html) ve [Integrator](https://www.mathworks.com/help/simulink/slref/integrator.html) belgeleri kullanıldı. Grafikler [figure Theme](https://www.mathworks.com/help/matlab/ref/matlab.ui.figure.html) özelliği bulunan sürümlerde açık temayla üretilir.
