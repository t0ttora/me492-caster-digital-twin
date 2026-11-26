# Bu kod dizininde nereden başlayacağım?

Önce `physics/motion.py` içindeki **evaluate_motion** fonksiyonunu aç. Bu bir
içindekiler sayfası gibidir: her satır fiziksel bir adımı çağırır. Henüz bütün
alt fonksiyonların cebrini bilmek zorunda değilsin.

| Sıra | Dosya ve giriş fonksiyonu | Cevapladığı soru |
|---|---|---|
| 1 | [state.py — State](state.py) | Hangi konum, açı ve hızlar değişiyor? |
| 2 | [parameters.py — load_parameters](parameters.py) | Kütleler, boyutlar, sürtünmeler hangi birimde? |
| 3 | [geometry.py — geometry_at](physics/geometry.py) | Gövde hareketi tekerlek merkezlerini nasıl taşır? |
| 4 | [support_loads.py — support_loads](physics/support_loads.py) | Ağırlığı hangi tekerlek ne kadar taşır? |
| 5 | [wheel_slip.py — wheel_slips](physics/wheel_slip.py) | Merkez hızı ile çevresel hızın farkı nedir? |
| 6 | [contact_forces.py — contact_forces](physics/contact_forces.py) | Kaymaya karşı hangi kuvvet/moment oluşur? |
| 7 | [inertia.py — mass_matrix_and_bias](physics/inertia.py) | Kütleler ve dönmeler birbirine nasıl bağlanır? |
| 8 | [energy.py — kinetic_energy / power_balance](physics/energy.py) | Enerji nerede depolanır ve kaybolur? |
| 9 | [motion.py — evaluate_motion](physics/motion.py) | M·ivme+bias=Q çözülünce hangi türevler çıkar? |
| 10 | [experiments.py — EXPERIMENTS](simulation/experiments.py) | Hangi deneyde neyi değiştiriyoruz? |
| 11 | [rk4.py — simulate](simulation/rk4.py) | Küçük adımlarla zamanı nasıl ilerletiyoruz? |
| 12 | [records.py — result_record](simulation/records.py) | B COM durumu A aks konumuna nasıl çevrilir? |
| 13 | [visualization.py](visualization.py) | Hesaplanan sayılar grafikte/animasyonda neyi gösterir? |

## İsimlerin sözlüğü

- `state`: o andaki konum, açı, hız ve birikmiş enerjiler.
- `parameters.geometry`: boyutlar; `parameters.dynamics`: kütle ve dirençler.
- `torques.left_Nm/right_Nm`: aks torkları; hız komutu değil.
- `normal_loads_N`: sol, sağ ve caster düşey yükleri.
- `slips`: her tekerlekte boyuna ve yanal kayma hızları.
- `contacts`: zemin kuvvetleri ve direnç momentleri.
- `matrix`: bağlı kütle matrisi M; `bias`: hızdan gelen merkezcil terimler.
- `force`: genel koordinatlardaki kuvvet/moment Q.
- `accelerations`: yedi hızın türevi; `derivative`: 13 durumun tüm türevleri.

`x_m` CSV'de **A aks merkezidir**. `body_x_m` Python durumunda **B COM'dur**.
Bu iki ismi aynı nokta sanma. İlk ders bu ayrımı çizerek anlatır.

## Küçük çalışma yöntemi

Her dosyada önce üstteki Türkçe açıklamayı, sonra tek giriş fonksiyonunu oku.
Bir kağıda “girdi → denklem → çıktı” yaz. Örneğin `wheel_slips` için:
“tekerlek merkezi hızı ve roll hızı → e·v-r·omega → kayma hızı”.
Bir dosyayı açıklayamıyorsan o dosyada kal; diğer 12 durumu birden ezberleme.

Alıştırmalar ve çözümler: [dört kısa ders](../../docs/learning/01-problem-ve-robot.md).
