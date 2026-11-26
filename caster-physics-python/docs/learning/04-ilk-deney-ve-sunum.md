# 4 · İlk deneyi çalıştır ve kendi cümlenle anlat

Önceki dersler: [robot](01-problem-ve-robot.md), [kod](02-kodu-okuma-sirasi.md),
[fizik](03-kuvvetlerden-harekete.md).

Terminali `caster-physics-python/` klasöründe aç. Kurulumun ana README'deki
adımları tamamlandığında ilk deney komutu:

```sh
.venv/bin/python run_experiments.py --group first-lesson
```

Bu komut iki durumu karşılaştırmak içindir: caster başlangıçta gövdeyle
hizalı ve başlangıçta 45° açılı. Başlangıç hızları ve motor tork programı
aynı tutulur. Böylece ilk caster yönünün etkisini inceleyebiliriz.

Çıktı adları `trajectory_comparison.png`, `angles_and_energy.png` ve
`robot_motion.html` olarak düzenlenir. Dosyaların yazıldığı klasörü komutun
çıktısından takip et. Bu dersin yazılmış olması, deneyin çalıştırıldığı veya
doğrulamanın geçtiği anlamına gelmez; çalıştırma sonucuna bak.

## Grafikleri bu sırayla oku

1. **Yörünge:** A (tahrik aksı orta noktası) nereden nereye gitti? Eksenler metre cinsinden mi?
   İki koşu gerçekten aynı noktadan mı başladı? X–Y grafiğinde eşit fiziksel
   mesafelerin eşit görünmesi gerekir; eksen ölçeği sapmayı abartmasın.
   Bu grafik `x_m/y_m` sütunlarını çizer. B gövde COM konumu ayrı
   `body_com_x_world_m/body_com_y_world_m` sütunlarında saklanır.
2. **Gövde yönelimi:** Robotun baktığı yön `theta` nasıl değişti? Yörüngedeki
   yan sapma ile gövdenin dönmesi aynı nicelik değildir.
3. **Caster açısı:** `phi`, gövdeye göre açıdır. “Caster odada nereye bakıyor?”
   sorusu için `psi = theta + phi` gerekir. Açı grafiğinin birimini kontrol et.
4. **Enerji:** Motor işi, kinetik enerji değişimi ve kayıp birbirini karşılıyor
   mu? Animasyonda güzel görünmesi bu kontrolün yerine geçmez.
5. **Animasyon:** Grafiklerde gördüğünü hareketle eşleştir. Üstten görünüş bir
   hesap görselleştirmesidir; gerçek robot videosu değildir.

İlk yorumun “açılı caster kesin şu kadar saptırır” olmasın. Önce grafikten
okuduğun olguyu söyle: “Bu parametreler ve bu tork programında iki koşunun
yörüngeleri şöyle ayrılıyor.” Sonra kuvvet ve moment mekanizmasını açıkla.

## MATLAB ile aynı mı? Ayrı komutla denetle

```sh
.venv/bin/python verify_model.py
```

Karşılaştırmanın iki farklı kapsamı var:

- **Üç deney:** Korunmuş MATLAB CSV kayıtlarıyla zaman boyunca durum ve
  hesaplanan tanı sütunlarının karşılaştırılması.
- **On iki deney:** MATLAB paketinde kayıtlı son konumlar ve caster açılarıyla
  son durum karşılaştırması. Bu, on iki tam zaman serisinin karşılaştırılması
  demek değildir.

Raporda hata büyüklüklerine, toleranslara ve başarısız koşullara bak. Ayrıca
korunan MATLAB kaynaklarının hash değerlerinin değişmediği denetlenir.
Bir dosyanın mevcut olması veya Python'un hata vermeden kapanması, tek
başına MATLAB ile eşdeğerlik kanıtı değildir.

Aynı sentetik denklemlerden aynı sayısal sonuçları almak, **taşımanın doğru
yapıldığına** kanıt sağlar. Gerçek robotun davranışını doğru tahmin ettiğimizi
göstermek için ölçüm ve kalibrasyon ayrıca gerekir.

## Hocaya iki dakikalık anlatım

1. **Soru:** “İki motora aynı tork verildiğinde caster'ın başlangıç yönünün
   hareketi nasıl etkilediğini inceliyorum.”
2. **Model:** “Gövdeyi, caster yönlenmesini ve üç tekerleğin dönmesini birlikte
   çözüyorum. Tekerlek hızını doğrudan emretmiyorum; tork, zeminle etkileşerek
   ivmeyi oluşturuyor.”
3. **Deney:** “İki koşuda aynı tork programını kullandım; başlangıç caster
   açısını değiştirdim.” Burada gerçekten çalıştırdığın grafiği göster.
4. **Mekanizma:** “Açılı caster'ın kayma yönü ve temas kuvveti değişiyor.
   Temas kuvvetinin moment kolu gövde ve caster hareketini birlikte etkiliyor.”
5. **Kontrol:** Doğrulama raporundaki **gerçek sonucu** söyle: hangi koşullar
   karşılaştırıldı, geçti mi, hata ne kadardı? Henüz çalıştırmadıysan bunu belirt.
6. **Sınır:** “Parametreler sentetik. Gerçek robotun ölçüleri ve deney kayıtları
   gelince parametreleri belirleyip öngörüleri ölçümle karşılaştıracağım.”

## Öğrenme ölçütü

İlk turda “Python biliyorum” demen gerekmiyor. Şu dört şeyi gösterebiliyorsan
modeli açıklama yolunda somut ilerledin:

- Başlangıç açısının ve tork programının kodda nerede olduğunu bulabiliyorsun.
- Tork ile hız arasındaki farkı söyleyebiliyorsun.
- Kayma → kuvvet → moment → ivme zincirini takip edebiliyorsun.
- Bir grafiğin neyi gösterdiğini ve hangi iddiayı henüz desteklemediğini ayırıyorsun.

### Bir dakikalık alıştırma

MATLAB karşılaştırması geçti ama gerçek robot verisi yok. “Model doğrulandı”
cümlesini tek başına kullanmak yerine ne söylersin?

<details><summary>Cevap</summary>

“Python'a taşıma, belirtilen MATLAB kayıtlarıyla belirlenen tolerans içinde
eşleşti. Parametreler sentetik; gerçek robotla davranış doğrulaması yapılmadı.”

</details>

[Önceki ders](03-kuvvetlerden-harekete.md) · [Başa dön](01-problem-ve-robot.md)
