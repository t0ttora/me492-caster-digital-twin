# MATLAB / CAD / Gazebo ortak sözleşmesi — me492-caster-v0.1

`parameters_si.json` tek sayısal kaynaktır. Değerler **provisional_synthetic**: ARCHE ölçüsü, kalibre edilmiş fiziksel parametre veya ürün spesifikasyonu değildir. CAD ya da Gazebo için ikinci bağımsız boyut tablosu oluşturulmaz; birim dönüştürme gerekiyorsa JSON'dan türetilir. MATLAB runner her çalıştırmada bu JSON'u okur ve kopyasını `run_manifest.json` içine yazar.

## Referans çerçevesi ve CAD

SI birimleri: metre, saniye, radyan. +X ileri, +Y sol, +Z yukarı. Gövde orijini tahrik aksının ortasının zemine izdüşümü; dünya zemin z=0. Pozitif yaw +Z sağ el kuralına uyar. `drive_track_m` tekerlek merkezleri arasındaki mesafedir; `caster_trail_m`, pivotun caster yuvarlanma yönünde tekerlek aksının önünde bulunduğu mesafedir.

| Alan / türetilen boyut | Değer (m) | Kullanım |
|---|---|---|
| chassis_size_m | [0,50; 0,32; 0,08] | x/y/z boyutları |
| chassis_center_m | [0; 0; 0,14] | gövde koordinatında şasi merkezi |
| şasi alt yüzeyi | z = 0,10 | center_z − size_z/2 |
| sol tahrik merkezi | [0; +0,19; 0,075] | +Y sol |
| sağ tahrik merkezi | [0; −0,19; 0,075] | −Y sağ |
| caster_pivot_m | [−0,18; 0; 0,10] | swivel pivotu, gövde koordinatında |
| caster axle düşümü | 0,06 | pivot_z − caster_radius |
| caster merkezi, phi=0 | [−0,23; 0; 0,04] | pivot gerisinde trail=0,05 |
| fork_yoke_gap_m | 0,026 | caster genişliği 0,020 için iç açıklık |
| fork_plate_thickness_m | 0,006 | temsili plaka kalınlığı |
| fork_stem_radius_m | 0,008 | temsili dikey stem yarıçapı |

Beş CAD parçası: şasi, sol tahrik tekerleği, sağ tahrik tekerleği, caster fork, caster tekerleği. Caster swivel ekseni +Z; tekerlek yuvarlanma ekseni, kendi yuvarlanma yönüne dik yatay eksendir. İleri hizalı konumda bu eksen +Y alınabilir. İçe aktarılan CAD/URDF/SDF yerel eksenleri veya sıfır açı ofsetleri farklıysa aktarım katmanı açık dönüşüm yapmalı. Bu JSON, M0 için kütle/atalet/sürtünme tanımlamaz; dinamik Gazebo çalıştırması ayrıca açıkça etiketlenmiş fiziksel parametreler gerektirir.

`chassis_*`, tekerlek genişlikleri, pivot yüksekliği ve fork alanları M0 hareket denklemlerini etkilemez; CAD geometri sözleşmesidir. M0 hareketinde aktif geometri: drive radius/track, caster radius/trail, pivot x/y.

## Komut CSV'si

Her dosya UTF-8, virgülle ayrılmış, noktalı ondalık sayılıdır. Başlık tam olarak:

```csv
time_s,linear_velocity_mps,angular_velocity_radps
```

Zaman 0'dan başlar ve kesin artar; tüm sayılar sonludur. İlk sürüm 0,02 s örnekler. Satırdaki komut bir sonraki örneğe kadar tutulur (**zero-order hold**). Son satır senaryo bitiş zamanıdır; bunun ötesinde deney sürdürülmez. `linear_velocity_mps` gövde +X hızı, `angular_velocity_radps` +Z etrafında yaw hızıdır. Tekerlek motor hızları değildir. Gazebo tarafı bu komutları tahrik eklem hızlarına çevirirken aynı radius/track kullanmalı. ROS topic adı veya taşıma gecikmesi bu şemada tanımlanmaz.

## Başlangıç durumu ve koşu kaydı

Komut CSV tek başına yeterli değildir. `run_manifest.json` içindeki her koşunun `scenario_id`, `duration_s`, `initial_pose` ve `initial_caster_angle_body_rad` alanları da aktarılır. Bütün başlangıç pozları [0,0,0]; caster başlangıcı senaryoya göre değişir. Robotun yerel caster sıfırı, phi=0'ın +X yuvarlanma yönüne karşılık gelmesiyle eşleştirilir.

## Sonuç CSV'si

Başlık tam olarak:

```csv
time_s,linear_velocity_mps,angular_velocity_radps,x_m,y_m,yaw_rad,caster_angle_body_rad,caster_angle_world_rad,caster_angle_body_rate_radps,left_wheel_rate_radps,right_wheel_rate_radps,caster_wheel_rate_radps,caster_hinge_x_m,caster_hinge_y_m,caster_wheel_center_x_m,caster_wheel_center_y_m
```

`x_m`, `y_m`, `yaw_rad` gövde orijininin dünya pozudur. `caster_angle_body_rad` gövdeye göre phi; `caster_angle_world_rad` psi = yaw + phi. Açılar sürekli ve radyandır; ±pi aralığına sarılmaz. Gazebo açıları sarılmış gelirse karşılaştırmada önce tutarlı unwrap yapılır. Üç wheel_rate alanı radyan/s, geometrik yuvarlanma yönüne göre işaretlidir; CAD eklem eksen işaretiyle eşleşmesi gerekir. Hinge ve wheel_center xy değerleri dünya koordinatındadır. Bütün sonuçlar komut zamanlarına örneklenir; MATLAB iç çözüm adımı 0,001 s'dir.

Gazebo sonucu MATLAB sonuçlarıyla karşılaştırılırken aynı başlangıç durumu, komut zamanları, eksen tanımları ve geometri kullanılır. Fark fiziksel temas modelinden veya sayısal çözümden kaynaklanabilir; aynı CSV kullanmak otomatik kalibrasyon değildir.
