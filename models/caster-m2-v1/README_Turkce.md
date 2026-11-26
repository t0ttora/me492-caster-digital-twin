# ME492 M2 v1 — tork girdili, kuvvetle bağlanmış düzlemsel model

Bu paket, kendi M0 kinematik modelimizden ayrı bir **dinamik Simulink modelidir**. Sol/sağ tahrik torkları giriş; gövde konumu, yönelimi ve tekerlek hızları sonuçtur. Caster temas kuvveti ve zemin bore momenti gövde ivmesini etkiler. Kütle, atalet, sürtünme ve direnç değerlerinin tamamı **sentetiktir**. Gerçek robot veya ARCHE CAD materyallerinden kalibrasyon yapılmadı.

M0 referansı korunur. `geometry_si.json`, M0'ın kanonik SI geometrisinin birebir kopyasıdır. Yeni fizik değerleri `dynamics_si.json` dosyasındadır. JSON içindeki `body_com_m`, şasinin geometrik merkezinden bağımsız seçilmiş kütle merkezidir. Toplam kütle 16,6 kg; bunun 16 kg'ı gövde/tahrik kütlesi olarak, 0,6 kg'ı caster aks merkezinde toplanmış kütle olarak modellenir. Fork ve caster tekerleği ayrı CAD cisimleri olsa da bu M2 onları tek caster kütlesi ve iki döner atalet ile temsil eder.

## İlk açılış ve tek deney

Paketi bütün klasör yapısıyla aç. MATLAB Current Folder'ı paketin `models` klasörüne getir; ardından:

```matlab
open_system('Me492CasterM2V1.slx');
```

Run düğmesine bas. İlk deney hizalı caster ile 4 saniyelik açık çevrim sürüştür. Modelin InitFcn'i üst klasördeki kaynakları MATLAB yoluna ekler; daha önce `addpath` çalıştırman gerekmez. SLX ile birlikte üst klasördeki `.m` dosyalarının bulunması gerekir. SLX'i tek başına başka yere taşımak yeterli değildir.

Doğrudan Run, SLX Model Workspace içinde kaydedilmiş parametre ve başlangıç durumlarını kullanır. Modelin InitFcn'i elle yaptığın Model Workspace değişikliklerini sıfırlamaz. JSON değişikliklerini deneylere uygulamak için paket kök klasöründen `run_m2` kullan.

## Tek senaryo ve tam doğrulama

MATLAB Current Folder'ı paket kök klasörüne getir:

```matlab
run_m2('misaligned_forward');  % yalnız 45 derece caster deneyi
```

Bütün deneyleri ve kontrolleri yeniden üretmek için:

```matlab
verify_m2('kernel');
run_m2;
verify_m2;
export_m2_results;
```

`run_m2`, JSON parametrelerini yeniden yükler. Kaydedilmiş kaynak SLX'in benzersiz, geçici bir kopyasını simüle eder. Açık kaynak modeli kapatmaz, kaydetmez veya senaryo değerleriyle değiştirmez. Kopya koşu sonunda temizlenir. Kaynak modelde kaydedilmemiş değişiklik varsa, hangi modelin çalıştırılacağı belirsizleşmesin diye açık bir hata verir; önce kendi değişikliklerini kaydet.

Koşuların CSV ve PNG dosyaları yeniden yazılır. Elle değiştirdiğin sonuçları başka klasörde sakla. Tek senaryo çalıştırmak manifesti yalnız o senaryoya günceller. Tam `verify_m2` için aynı nesilden 12 senaryo ve komut CSV'si gereklidir; eksik veya değiştirilmiş dosya tespit edilirse durur.

`dynamics_si.json` içindeki `command_sample_dt_s` komutların, `result_sample_dt_s` ise sonuç CSV'sinin zaman aralığıdır. Varsayılan ikisi de 0,01 s'dir; bağımsız seçilebilirler. Komutlar sıfırıncı derece tutmayla uygulanır. Sabit `ode4` çözüm adımı varsayılan 0,0005 s'dir. Komut/sonuç aralıkları bunun tam katı olmalıdır; çözüm adımıyla örnekleme aralığı aynı kavram değildir.

`verify_workflow`, temiz MATLAB yolundan `models` klasöründe açılışı, 13 durumu, açık modelin korunmasını, kaydedilmiş elle düzenlemenin aynı kalmasını, 10 ms komut / 20 ms sonuç ayrımını, kaydedilmemiş değişiklikte durmayı ve geçici kopyanın temizliğini ayrı test paketinde kontrol eder. Aynı adlı, kaydedilmemiş eski model açıkken builder'ın onu koruması ve yeniden kurulan modelin derlenip çalışması da test edilir. Bunu ayrı MATLAB batch oturumunda çalıştır; açık kullanıcı oturumunda test modeliyle aynı adları yüklemeye çalışma.

## Simulink akışını okumak

1. **IndependentWheelTorques:** sol ve sağ tekerlek için iki bağımsız tork girdisi. Eşit tork, eşit tekerlek hızı veya düz yol garantisi değildir; deneyler açık çevrimdir.
2. **WheelGroundForces:** çift tıkla. `SupportLoadBalance` destek yüklerini, `WheelSlipVelocities` boyuna/yanal kaymayı, `FrictionAndResistances` pasif temas kuvvetlerini ve dirençleri hesaplar. `ContactDiagnostics` yalnız kayıt için tanı sinyali üretir.
3. **BodyCasterInertiaDynamics:** çift tıkla. `MassMatrixAndBias` bağlı kütle matrisini ve geometrik hız terimlerini; `GroundAndDriveForces` genelleştirilmiş kuvveti oluşturur. `CoupledAcceleration`, `M * nuDot + bias = Q` denklemini çözer. `PowerAndEnergy` güçleri ve iki enerji integralinin türevlerini hesaplar.
4. **MechanicalState:** bu türevleri entegre eder. Durum bilgisi alt taraftaki geri dönüş hattıyla temas ve atalet hesaplarına tekrar girer. Enerji muhasebesi hareketi zorlamaz.
5. **StateRecord / ForceRecord / ContactRecord / EnergyPowerRecord:** Simulink sonucunu kaydeder. `run_m2`, bu kayıtları SI birimli CSV'ye dönüştürür. Diyagramdaki MATLAB Function aşamaları kendi `.m` kaynaklarına çağrı yapar; denklemler `theory.md` içindedir.

### 11 fiziksel durum + 2 enerji integrali

| İndeks | Durum | Birim ve referans |
|---|---|---|
| 1–2 | B_x, B_y | gövde kütle merkezi, dünya m |
| 3 | theta | gövde yaw, rad; pozitif CCW |
| 4 | phi | caster'ın gövdeye göre açısı, rad |
| 5–6 | V_x, V_y | gövde COM dünya hızı, m/s |
| 7 | Omega | gövde yaw hızı, rad/s |
| 8 | rho | göreli caster swivel hızı, rad/s |
| 9–11 | omega_L, omega_R, omega_C | üç tekerleğin roll hızı, rad/s |
| 12–13 | W_motor, D_loss | motor işi ve sönümlenen enerji, J |

Dünya caster açısı `psi = theta + phi`; mutlak swivel hızı `Omega + rho` olur. Dünya +X/+Y; gövde +x ileri, +y sol, +z yukarıdır. A tahrik aksının zemine izdüşen orta noktası, B gövde COM, H caster pivotu, C caster aks merkezidir. CSV'deki `x_m/y_m` A'yı, integratörün ilk iki bileşeni B'yi gösterir. Birbiriyle karıştırma. 32 parametrenin SI birimleri ve durum haritası `state_parameter_map.json` dosyasındadır.

Bu sürüm açık düzlemsel denklemlerle çalışan Simulink modelidir. Simscape Multibody kullanılmadı; 3B çok cisimli model olarak sunulmaz. Onshape montajındaki parça sayısı, bağımsız dinamik cisim sayısı değildir. Görsel CAD parçalarını ayrı fiziksel durumlara çevirmek veya renk/malzeme etiketlerinden kütle çıkarmak bu sürümün kapsamı değildir.

`build_m2`, paket içindeki `models/Me492CasterM2.slx` başlangıç modelini kopyalayarak yeni V1'i kurar. Bu dosya builder girdisidir; normal çalışma için `Me492CasterM2V1.slx` açılır. Builder benzersiz bir çalışma kopyası kullanır; açık eski modeli yeniden adlandırmaz veya kapatmaz. Builder mevcut V1'i üzerine yazmaz. Yeniden kurmak istiyorsan önce kendi V1 düzenlemelerini başka bir isimle koru; mevcut V1 dosyası bulunurken builder açık hata verir.

## Koşular

| Koşu | Kontrol ettiği davranış |
|---|---|
| aligned_forward / misaligned_forward | aynı tekerlek torku; caster başlangıcı 0° / 45° |
| mirror_misaligned | −45° başlangıçla x/yaw işaret simetrisi |
| caster_contact_disabled | **caster'ın teğetsel temas etkileri kapalı**; kütle, atalet, düşey destek ve bearing korunur |
| unequal_torque | sağ tork daha büyükken sol yönde dönüş |
| reversal_perturbed | 1° ilk caster açısı, ileri/geri tork geçişi |
| reversal_no_swivel_resistance | aynı ters sürüş girdisi, bearing ve bore direnci sıfır; duyarlılık karşılaştırması |
| zero_rest | sıfır torkta duran robotun kendiliğinden hareket etmemesi |
| passive_coast | sıfır torkta başlangıç kinetik enerjisinin sönmesi |
| conservative_free | sıfır temas sürtünmesi/direnç; enerji ve toplam momentum korunumu |
| misaligned_fine / misaligned_finer | h, h/2, h/4 çözüm adımı karşılaştırması |

Caster kapalı karşılaştırması üçüncü tekerleği fiziksel olarak kaldırmaz. Aynı destek yükleri ve hareket eden caster kütlesi korunur; caster zemin kayma kuvveti, bore momenti ve caster yuvarlanma direnci sıfırlanır. Bu, ideal sürtünmesiz caster desteği ile matematiksel bir karşılaştırmadır.

Ters sürüşte caster'ın mutlaka 180° dönmesi genel bir fizik kuralı değildir. Tekerlek geriye yuvarlanabilir; dirençler, gövde yaw hareketi ve tork girdileri caster'ın dönmesini değiştirebilir. İlk doğrulamada zorunlu 180° varsayımı sonuçlarla çelişti. Bu varsayım düzeltilerek aynı girdili direnç duyarlılık koşusu eklendi; koşunun sonucu modelde zorla değiştirilmedi.

## Fiziksel kapsam ve sınırlamalar

- Gövde düzlemde x/y/yaw bakımından serbesttir; caster pasif swivel ve roll ataletine sahiptir. Üç tekerlekte boyuna/yanal kayma mümkündür.
- Düşey konum, roll ve pitch sabittir. Düşey darbe, süspansiyon, tekerlek kalkması, devrilme ve eğim simüle edilmez. Normal kuvvetler, caster açısına göre anlık destek üçgeninden **quasi-static** olarak hesaplanır; ivmeye bağlı yük transferi yoktur.
- Caster fork/tekerlek kütlesi aks merkezine toplanır. Caster swivel ataleti bu kütle merkezinin çevresindeki atalet; trail kaynaklı ötelenme ataleti ayrıca geometriyle hesaplanır. Tahrik tekerleklerinin öteleme kütlesi gövde kütlesindedir; roll ataletleri ayrı tutulur.
- Kayma sürtünmesi ve bore torku düzenlenmiş, enerji tüketen bir modeldir. Caster kuvveti ile bore momenti ortak elipsoidal sürtünme bütçesini paylaşır. Hertz/Kalker temas çözümü veya gerçek kauçuk modeli değildir.
- `tanh` ile bearing/rolling dirençleri ve yumuşak temas modeli **tam statik kilitlenme/stiction** çözmez; sıfıra yakın kayma/creep mümkündür. Shimmy tahmini iddiası yoktur.
- CAD malzeme isimleri veya renkleri bu sentetik kütle/ataletleri doğrulamaz. Robot hız kontrolcüsü, motor elektrik devresi, ROS gecikmeleri veya NMPC bu M2'ye eklenmedi.

Doğrulama, bu açık varsayımlar altında işaretleri, enerji/momentum dengesini, temas kapasitesini ve sayısal yakınsamayı gösterir. Robot verisine karşı geçerleme veya araştırma hipotezinin sahada kanıtı değildir. Büyük açık çevrim yaw sapmaları gerçek robotta beklenen performans olarak sunulmamalı; ölçülen kontrol döngüsü ve parametrelerle yeniden değerlendirilmelidir.

## Yerel yürütme kanıtları

7 Ekim 2026 tarihli native MATLAB R2026b batch yürütmesi **exit 0** ile tamamlandı: 12 senaryo, **60/60 fizik ve sayısal kontrol**, ayrı **17/17 çekirdek kontrolü** ve **13/13 çalışma akışı kontrolü** başarılı. Varsayılan 12 sonuç CSV'si önceki M2 v0 çıktılarıyla byte düzeyinde aynı; refaktör fizik davranışını değiştirmedi. En büyük enerji denge hatası 3,8019e−7 J; h–h/2 yaw farkı 1,9689e−10 rad.

Yeni sürümün sonuçları `executed_summary.json`, `verification_evidence.json`, `kernel_verification_evidence.json`, `workflow_verification_evidence.json` ve başarılı native transcript dosyalarında kayıtlıdır. Koşu çıktıları gerçek MATLAB R2026b / Simulink yürütmesinden gelir. Sayısal ve fiziksel tutarlılık kontrolleri, gerçek robot doğrulaması anlamına gelmez.

## Koruma ve aktarım kapsamı

Orijinal M0 ve M2 v0 klasörleri değiştirilmedi. İşleme başlamadan disk dosyaları, açık MATLAB modelinin durumu ve model kopyası ayrı `preservation_20261007_221034` klasörüne alındı. Denetimde kaynak model kaydedilmişti; açık editörde kaydedilmemiş belge yoktu. Revizyon ayrı `caster_model_m2_v1` klasöründe üretildi. `capability_audit.json` denetimi, `preservation_verification.json` son hash karşılaştırmasını içerir.

Windows çalışma klasöründe 12 sonuç CSV'si, 12 komut CSV'si ve bütün grafikler bulunur. `m2_v1_review.zip` inceleme paketi kaynakları, iki SLX'i (builder girdisi ve V1), parametreleri, belgeleri, native kanıtları, üç Simulink diyagramını, iki okunabilir sonuç grafiğini ve yalnız üç temsili sonuç CSV'sini içerir: `aligned_forward`, `misaligned_forward`, `passive_coast`. Manifestin diğer CSV yolları ZIP içinde mevcut değildir. Tam tekrar ve doğrulama için kök klasörde `run_m2; verify_m2;` çalıştır.

Native CSV nesil kontrolü, Java istemeyen `Simulink.getFileChecksum` MD5 değerleriyle yapılır. Aktarım ZIP'i ve üye dosyalar için ayrıca SHA-256 kaydedilir. CSV checksum'ları güvenlik garantisi olarak kullanılmaz. Mac'e paket aktarımı, Mac'te Simulink'in yürütüldüğü anlamına gelmez. Video üretimi ayrı aşamadır.

## Referanslar

CSV eksenleri `schema.md`; denklemler ve kaynakları `theory.md` açıklar. Model yaklaşımı için iki birincil kaynak kullanıldı: [Schröder, Lenord ve Lange, 2019](https://2019.international.conference.modelica.org/proceedings/html/papers/Modelica2019paper4B2.pdf), düzlemsel tahrik/caster ve bore direnci modellemesi; [Jaswin ve Indrawanto, 2020](https://arxiv.org/pdf/2011.01066), kaymadan türetilen tekerlek kuvveti ve tahrik atalet denklemleri. Lagrange/Jacobian modeli kendi geometri varsayımlarımızdan türetildi; kaynak modellerin veya robot geçerlemesinin kopyası değildir.

Simulink uygulaması için [MathWorks MATLAB Function](https://www.mathworks.com/help/simulink/slref/matlabfunction.html), [model geri çağırımları](https://www.mathworks.com/help/simulink/ug/model-callbacks.html) ve [Simulink.getFileChecksum](https://www.mathworks.com/help/simulink/slref/simulink.getfilechecksum.html) belgeleri kullanılır.
