# 1 · Biz hangi soruyu çözüyoruz?

Bu modelin sorusu şu: **Motorlara aynı torku verdiğimizde, caster'ın ilk
yönü robotun hareketini nasıl değiştiriyor?**

Burada *caster*, kendi ekseni etrafında yuvarlanan ve ayrıca dikey bir mil
etrafında dönebilen serbest tekerlek. İki motorlu tekerlek robotu sürüyor;
caster ise gövdeye bağlı mafsalından yönleniyor. Bu model düzlemde hareketi
hesaplıyor: ileri/yanal hareket ve üstten bakıldığında dönme var. Robotun
zıplaması, gövdenin öne eğilmesi veya yana devrilmesi çözülmüyor.

## Önce iki noktayı ayır

- **A:** Sol ve sağ motorlu tekerleğin arasındaki aks merkezi. Gövdeye ait
  geometrik ölçülerin referansı.
- **B:** Ana gövdenin kütle merkezi (*COM*). Modelin `body_x_m` ve
  `body_y_m` durumlarının izlediği nokta.

A ve B aynı olmak zorunda değil. B'nin A'ya göre konumu bir **geometri
parametresi**. B'nin dünya üzerindeki konumu ise zamanla değişen bir **durum**.
Kütle merkezini aks merkeziyle karıştırırsak moment kollarını da yanlış kurarız.
Caster kütlesi ayrıca modellenir; B, bütün robotun birleşik kütle merkezi değildir.

**H** caster'ın gövdeye bağlandığı dikey mafsal, **C** ise modelde caster'ın
zemindeki konumunu ve toplu kütlesini temsil eden noktadır. C, H'nin caster'ın
yuvarlanma yönüne göre gerisindedir. Bu mesafeye *trail* denir. Mafsal ile temas
noktası arasındaki bu kol, temas kuvvetinin caster'ı yönlendirebilmesini sağlar.

## Üç açı, üç farklı anlam

| Sembol | Anlam | Basit örnek |
|---|---|---|
| `theta` / `body_yaw_rad` | Gövdenin dünyaya göre yönü | Robot odanın sağına bakıyor. |
| `phi` / `caster_angle_relative_rad` | Caster'ın gövdeye göre yönü | Caster robota göre 45° dönük. |
| `psi = theta + phi` / `caster_world_angle_rad` | Caster'ın dünyaya göre yönü | Gövde 30°, caster göreli 45° ise 75°. |

Pozitif açılar üstten bakınca saat yönünün tersine artar. Kod içinde açılar
**radyan**, kullanıcıya gösterilen grafiklerde derece olabilir. `phi = 0`,
caster'ın yuvarlanma yönünün gövdenin ileri yönüyle hizalı olması demektir.

`body_yaw_rate_radps`, `theta` açısının değişim hızıdır.
`caster_relative_rate_radps`, `phi` açısının değişim hızıdır. Caster'ın
dünyaya göre dönme hızı `caster_world_rate_radps`, bu ikisinin toplamıdır.
Tekerleğin yuvarlanma hızı `caster_roll_rate_radps` bunlardan ayrıdır.

## Girdi, durum, sonuç

**Girdi:** Sol ve sağ motora uygulanan torklar; birimleri N·m. **Başlangıç:**
konumlar, açılar ve hızlar. **Parametreler:** kütleler, boyutlar, ataletler ve
sürtünme katsayıları. **Sonuç:** Bunların zamanla nasıl değiştiği.

Tork komutu, hız komutu değildir. Motora tork verince tekerlek ivmelenir.
Zeminle etkileşim, tekerlek ve robotun hızlarını birlikte belirler. Bu modelde
“1 m/s git” diyen bir hız kontrolcüsü yoktur.

Parametreler henüz **sentetik**: gerçek robot ölçümleriyle belirlenmiş değiller.
Modeli öğrenebilir ve sayısal tutarlılığını sınayabiliriz; buradan gerçek robotun
kaç santimetre sapacağını doğrulanmış sonuç olarak çıkaramayız.

### Bir dakikalık alıştırma

Gövde dünyaya göre 20°, caster gövdeye göre −10° dönük. Caster dünyaya göre
kaç derece? Robotun konum grafiğindeki nokta A mı B mi?

<details><summary>Cevap</summary>

`psi = 20° − 10° = 10°`. Yörünge grafiği A tahrik aksı orta noktasını
(`x_m/y_m`) izler. B gövde COM konumu çözücüde entegre edilir ve ayrı
`body_com_x_world_m/body_com_y_world_m` sütunlarında saklanır.

</details>

Devam: [2 · Kodu hangi sırayla okuyacağım?](02-kodu-okuma-sirasi.md)
