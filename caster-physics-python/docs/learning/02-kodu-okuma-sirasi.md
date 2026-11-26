# 2 · Kodu hangi sırayla okuyacağım?

Önce [robotu ve açıları](01-problem-ve-robot.md) oku. Sonra bütün dosyaları aynı
anda anlamaya çalışma. Aşağıdaki sıra, bir deneyin nasıl kurulduğunu gösterir.
Dosya yolları `caster-physics-python/` klasörüne göredir.

## Kod dizini

```text
run_experiments.py                 Deneyi çalıştır, grafikleri hazırla.
verify_model.py                    Referansla karşılaştır, kaynakları denetle.
src/caster_physics/
├── state.py                       Robot şu an nerede, nasıl hareket ediyor?
├── parameters.py                  Robotun hangi özelliklerini sabit kabul ettik?
├── physics/
│   ├── geometry.py                Noktalar, yönler, moment kolları.
│   ├── support_loads.py           Ağırlık hangi tekerleğe ne kadar biniyor?
│   ├── wheel_slip.py              Tekerlek zemine göre nasıl kayıyor?
│   ├── contact_forces.py          Bu kaymaya zemin nasıl karşılık veriyor?
│   ├── inertia.py                 Bağlı kütlelerin ve dönmelerin ataleti.
│   ├── energy.py                  Giren iş ve kaybolan enerji hesabı.
│   └── motion.py                  Kuvvetlerden durum türevlerini hesapla.
└── simulation/
    ├── experiments.py             Başlangıçlar, tork programı, süre.
    ├── rk4.py                     Küçük zaman adımlarıyla ilerle.
    └── records.py                 Sonuçları okunabilir sütunlara kaydet.
```

`physics/` **hesabı**, `simulation/` bu hesabın **hangi koşullarda ve ne kadar
süre çalışacağını** belirler. Grafiği değiştirmek fizik denklemini değiştirmez.
Tork programını değiştirmek deney koşulunu değiştirir.

## İlk okuma turu: deneyi tanı

1. [`experiments.py`](../../src/caster_physics/simulation/experiments.py):
   Önce hizalı ve açılı başlangıcı bul. Başlangıç caster açısı dışında hangi
   ayarların aynı tutulduğuna bak. Tork zamanla nasıl değişiyor?
2. [`parameters.py`](../../src/caster_physics/parameters.py): `Geometry`,
   tekerlek boyutlarını ve mafsal konumunu; `Dynamics`, kütle/atalet/sürtünme
   özelliklerini, gövde kütle merkezi ofsetini ve zaman adımlarını;
   `Parameters` bunları bir arada taşır. Çalışan sayılar `parameters/` içindeki
   SI JSON dosyalarından okunur. Birimi görmeden sayıyı yorumlama.
3. [`state.py`](../../src/caster_physics/state.py): `State`, bir anın fotoğrafı.
   Konum, gövde açısı, göreli caster açısı ve hızlar isimleriyle erişilir.

Durumda 13 sayı vardır: **4 konum/açı + 7 hız + 2 enerji hesabı**. Son iki
sayı motorun yaptığı işi ve biriken enerji kaybını izler; ayrı fiziksel
serbestlik dereceleri değillerdir. “13 durum” demek “13 bağımsız hareket yönü”
demek değildir.

Sayısal çözücü bu isimli bilgileri bir sayı dizisine dönüştürmek zorundadır.
Bu sıra `state.py` içinde tutulur. Fizik okurken `x[7]` gibi bir sayının ne
olduğunu ezberlemek yerine isimli alanı takip et.

İsimlerdeki ekler birimleri hatırlatır: `_m` metre, `_rad` radyan, `_mps`
metre/saniye, `_radps` radyan/saniye, `_J` joule.

| Aradığın bilgi | `State` içindeki alan |
|---|---|
| B'nin dünyadaki konumu | `body_x_m`, `body_y_m` |
| Gövde ve göreli caster açısı | `body_yaw_rad`, `caster_angle_relative_rad` |
| B'nin dünyadaki hızı | `body_velocity_x_mps`, `body_velocity_y_mps` |
| Gövde ve göreli caster dönme hızı | `body_yaw_rate_radps`, `caster_relative_rate_radps` |
| Üç tekerleğin yuvarlanma hızı | `left_wheel_rate_radps`, `right_wheel_rate_radps`, `caster_roll_rate_radps` |
| Biriken iş ve enerji kaybı | `motor_work_J`, `dissipated_energy_J` |

`body_velocity_x_mps`, robotun kendi ileri yönündeki hız değildir; dünyanın
X eksenindeki bileşendir. Robot dönerse bu ayrım önem kazanır. `State.at_rest`
başlangıcında A noktası `(0, 0)`, B ise kütle merkezi ofsetindedir; B'nin
koordinatları bu yüzden sıfır olmak zorunda değildir.

## İkinci tur: bir hesap adımını izle

[`motion.py`](../../src/caster_physics/physics/motion.py) dosyasını kavşak gibi
oku; çağırdığı dosyaları şu sorularla aç:

**Geometri → yükler → kayma → temas kuvvetleri → atalet → ivme.**

Bu zinciri başlatan fonksiyon `evaluate_motion(state, torques, parameters)`.
`torques`, `Torques(left_Nm=..., right_Nm=...)` biçiminde iki motor torkunu
taşır. Dönen `MotionEvaluation` içinde `derivative` durumun değişim hızlarını,
`contacts` temas kuvvetlerini, `slips` kaymaları, `normal_loads_N` destek
yüklerini içerir. Yani yalnız son yörüngeye bakmak zorunda değilsin;
hareketin nedenini ara fizik sonuçlarından da takip edebilirsin.

Bir sonraki ders bu zinciri denklemlere bağlar. İlk turda bütün matris
elemanlarını türetmeye çalışma; her dosyanın **ne aldığını, ne verdiğini ve
birimini** söyleyebilmen yeterli.

Sonra [`rk4.py`](../../src/caster_physics/simulation/rk4.py) dosyasına geç.
Burada yeni bir fizik kanunu yok: modelin verdiği türevleri kullanarak bir
sonraki ana ilerliyoruz. Her adımda dört ara değerlendirme yapılıyor. Tork
komutu, mevcut MATLAB deneyindeki gibi bir adım boyunca tutuluyor.

## Neyi nerede değiştireceğim?

| Merak ettiğin değişiklik | İlk bakacağın yer |
|---|---|
| Caster ilk anda 45° yerine 30° olsun. | `simulation/experiments.py` |
| Motor torkunu veya deney süresini değiştir. | `simulation/experiments.py` |
| Gerçek robotun tekerlek yarıçapını gir. | `parameters/geometry_si.json` içindeki `drive_radius_m` veya `caster_radius_m` |
| Temas/sürtünme varsayımını incele. | `physics/contact_forces.py` |
| Aynı denklemleri daha küçük zaman adımıyla çöz. | Deney ayarı ve `simulation/rk4.py` |

İlk öğrenme turunda referans deneyleri değiştirme. Yeni deneyi ayrı bir adla
eklemek, karşılaştırma dayanağını korur.

### Bir dakikalık alıştırma

“Caster'ın açısını değiştirince yörünge neden değişiyor?” sorusunda hangi
dosyada başlangıcı değiştirir, hangi üç aşamada mekanizmayı ararsın?

<details><summary>Cevap</summary>

Başlangıç `experiments.py` içinde. Mekanizma için `geometry.py` ile yönlere ve
moment kollarına, `wheel_slip.py` ile kaymaya, `contact_forces.py` ile zeminin
kuvvetine bak; `motion.py` bunları ivmeye bağlar.

</details>

[Önceki ders](01-problem-ve-robot.md) · [3 · Kuvvetlerden harekete](03-kuvvetlerden-harekete.md)
