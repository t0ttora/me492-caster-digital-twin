# 3 · Kuvvetlerden hareket nasıl çıkıyor?

Bu dersi kodla yan yana aç. Amaç her matris elemanını ezberlemek değil,
**hesabın nereden başlayıp nereye gittiğini** görebilmek.

## 1. Noktaların hızını bul — `geometry.py`

Gövde kütle merkezi B'nin hızını bilmek, caster C'nin hızını tek başına vermez.
Gövde dönerken B'den uzaktaki noktalar ayrıca hareket eder. Caster mafsalı da
döndüğünde C'ye bir hız daha eklenir.

Bir gövde noktası için temel ilişki:

`nokta hızı = B'nin hızı + gövdenin dönmesinden gelen hız`.

İkinci terim, düzlemsel `omega × r` ilişkisidir. Caster için göreli mafsal
dönmesinin katkısı da eklenir. Kod bunları `v = J ν` biçiminde toplar. Burada
`J` hızları nokta hızına çeviren matristir; yeni bir fizik varsayımı değildir.
Kodda `geometry_at(...)` bu yönleri ve hız matrislerini verir.

## 2. Normal yükleri bul — `support_loads.py`

Üç zemin desteğinin yükleri `N_L`, `N_R`, `N_C` olsun. Dikey kuvvet dengesi:

`N_L + N_R + N_C = (m_body + m_caster) g`.

A etrafındaki moment dengeleri de kullanılarak yükler paylaştırılır. Caster'ın
konumu döndükçe bu paylar değişebilir. Bu hesap **yarı statiktir**: her anda
dikey denge var kabul edilir. İleri ivmeden kaynaklanan dinamik yük aktarımı,
gövdenin yatması ve tekerleğin zeminden kopması bu modelde çözülmez. Pozitif
yük şartı bozulursa modelin destek varsayımı geçerli değildir.
Bu adımı yapan fonksiyon `support_loads(...)`; sonuç sırası sol, sağ, caster.

## 3. Kaymayı bul — `wheel_slip.py`

Bir tekerleğin yuvarlanma yönündeki kayma hızı:

`s_long = tekerlek merkezinin o yöndeki hızı − yarıçap × yuvarlanma hızı`.

Yanal kayma, merkez hızının tekerlek düzlemine dik bileşenidir. Örneğin merkez
0,30 m/s ilerliyor ve `r × omega = 0,30 m/s` ise boyuna kayma sıfırdır.
`r × omega = 0,40 m/s` ise boyuna kayma −0,10 m/s olur.

Kayma dışarıdan keyfî eklenen bir itki değildir; mevcut hareketin ve dönmenin
zeminle uyuşmayan kısmıdır. Açılı caster'ın yuvarlanma ve yanal yönleri de
açılıdır; bu yüzden aynı gövde hareketi farklı kayma bileşenleri üretir.
`wheel_slips(...)` sonucundaki `left_mps`, `right_mps`, `caster_mps` alanlarının
her biri `[boyuna, yanal]` biçiminde iki hız taşır.

## 4. Zeminin cevabını bul — `contact_forces.py`

Temas kuvveti kaymaya karşı koyar. Küçük kaymada sönüm benzeri bir ilişki
kullanılır; kuvvet büyüdükçe sürtünme kapasitesiyle yumuşak biçimde sınırlanır.
Sınır normal yük `N` ve sürtünme katsayısı `mu` ile ilişkilidir. Caster'ın
zeminde kendi etrafında dönmesine karşı momenti de aynı temas kapasitesini
paylaşır. Model, tam yapışmayı bir kısıt olarak dayatan ideal yuvarlanma modeli
değildir; küçük kayma oluşabilir.
`contact_forces(...)` hem tekerlek yönlerindeki hem dünya yönlerindeki kuvvetleri
ayrı isimlerle döndürür. Moment hesaplarken vektörleri aynı koordinat
sisteminde kullanmak gerekir.

İki farklı direnci ayır:

- **Mafsal yatağı direnci:** Caster ile gövdenin birbirine göre dönmesine
  karşıdır. Göreli hız `phi_dot` ile ilişkilidir; sistemin iç etkileşimidir.
- **Zemin üzerinde burulma direnci (*bore*):** Temasın zemine göre dönmesine
  karşıdır. Mutlak hız `theta_dot + phi_dot` ile ilişkilidir; dış momenttir.

Bu ayrım, momentin hareket denkleminin hangi bileşenine gireceğini belirler.

## 5. Bütün parçaları birlikte ivmelendir — `inertia.py`, `motion.py`

Tek bir nokta için `F = m a` yazabiliyoruz. Burada gövde, caster ve tekerlek
dönmeleri birbirine bağlı. Bunun bir arada yazılmış hâli:

**`M ν_dot + b = Q`**, dolayısıyla **`M ν_dot = Q − b`**.

| Sembol | Bu modelde ne demek? |
|---|---|
| `ν` | B'nin iki doğrusal hızı; gövde ve göreli caster dönme hızı; üç tekerleğin yuvarlanma hızı. Toplam 7 hız. |
| `M` | Kütleleri, dönme ataletlerini ve aralarındaki bağları taşıyan 7 × 7 matris. |
| `b` | Geometri ve mevcut dönme hızlarından gelen merkezcil katkılar. |
| `Q` | Zemin kuvvetleri, motor torkları ve dirençlerin bu 7 hareket bileşenine etkisi. |

`Q` karışık bir vektördür: doğrusal hareket satırları N, dönme satırları N·m
taşır. Bütün elemanları aynı birimdeymiş gibi toplamayız. Kod `M`'nin tersini
elle yazmaz; doğrusal denklem sistemini çözer.
`mass_matrix_and_bias(...)` fonksiyonu `M` ile `b`'yi;
`generalized_forces(...)` fonksiyonu `Q`'yu kurar. `evaluate_motion(...)`
bu sonuçlarla `np.linalg.solve(matrix, force-bias)` satırında ivmeleri bulur.

Aynı temas kuvveti hem gövdeyi hem caster'ı hem tekerleğin dönmesini etkiler.
Gövdeye ayrı, tekerleğe bağımsız bir “sürtünme düzeltmesi” eklemiyoruz. Bu,
etkileşimlerin ve enerji hesabının tutarlı olması için önemlidir.

### Küçük kuvvet–moment örneği: yalnızca öğretmek için

Bu sayılar **gerçek simülasyonun girdileri veya sonuçları değil**. Varsayımsal
bir anda B'den temas noktasına vektör `(r_x, r_y) = (−0,20; 0) m`, zeminin
uyguladığı kuvvet `(F_x, F_y) = (0; 5) N` olsun.

`M_z = r_x F_y − r_y F_x = (−0,20)(5) − 0 = −1 N·m`.

Negatif moment saat yönündedir. Caster'ın yanal kuvvetinin gövdeyi nasıl
döndürebileceğini bu örnek gösterir. Bu hesabı tüm bağlı robotun ivmesine
çevirmek için `M ν_dot = Q − b` çözülmelidir; tek başına `−1 / I_body`
hesabı tam modelin cevabı değildir.

## 6. Zamanı ilerlet, enerjiyi kontrol et — `rk4.py`, `energy.py`

İvmeleri küçük zaman aralıklarında kullanınca hızlar; hızları kullanınca konum
ve açılar değişir. RK4 bu sayısal ilerlemenin yöntemidir. Zaman adımını
küçültünce sonucun yakınsaması gerekir.

Motor gücü `P_motor = tau_L omega_L + tau_R omega_R`. Beklenen hesap:

`kinetik enerji değişimi ≈ motorların yaptığı iş − biriken enerji kaybı`.

Küçük fark sayısal integrasyondan gelebilir; büyüyen fark araştırılmalıdır.
Bu enerji kontrolü tutarlılık kanıtıdır, gerçek robotla kalibrasyon değildir.

### Bir dakikalık alıştırma

Gövde 0,5 rad/s dönüyor, caster gövdeye göre −0,5 rad/s dönüyor. Mafsalın
göreli hareketi var mı? Caster'ın zemine göre açısal hızı ne?

<details><summary>Cevap</summary>

Evet, `phi_dot = −0,5 rad/s`: yatak direnciyle ilişkili göreli hareket var.
`psi_dot = 0,5 − 0,5 = 0`: bu anda caster'ın mutlak yönü değişmiyor; bore
modelinin açısal hız girdisi sıfır. Caster'ın öteleme kayması yine olabilir.

</details>

[Önceki ders](02-kodu-okuma-sirasi.md) · [4 · İlk deney ve sunum](04-ilk-deney-ve-sunum.md)
