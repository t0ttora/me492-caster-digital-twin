# M2 SI veri sözleşmesi

`geometry_si.json` M0 kanonik geometrisinin değişmeden kopyası; `dynamics_si.json` sentetik fizik parametre kaynağı. `reference_hashes.json` M0 model/geometri hash'lerini saklar. M2 yeni bir şemadır; M0 v/omega komut dosyaları M2 tork dosyaları olarak yorumlanamaz.

Geometri kopyasında bulunan M0 `command_sample_dt_s` ve `solver_step_s` alanları M2 yürütmesinde kullanılmaz. M2 örnekleme ve çözüm adımı yalnız `dynamics_si.json` ve koşu manifest'inden alınır.

## Çerçeveler ve durumlar

Ortak robot orijini **A**: tahrik aksı orta noktasının zemine izdüşümü. +X ileri, +Y sol, +Z yukarı; pozitif yaw +Z sağ el kuralı. Dünya düz zemini z=0.

Dinamik integratör ilk iki koordinatı **B gövde COM dünya konumu** olarak tutar. JSON `body_com_m` B'nin A'ya göre gövde koordinatıdır. Çıktı dönüşümü açıkça:

```text
A_world = B_world - R(yaw)*body_com_xy
x_m = Bx - cos(yaw)*cx + sin(yaw)*cy
y_m = By - sin(yaw)*cx - cos(yaw)*cy
```

CSV `x_m,y_m,yaw_rad` **tahrik aksı orijininin pozudur**; `body_com_x_world_m,body_com_y_world_m` B koordinatlarını ayrıca taşır. `body_long_velocity_mps,body_lat_velocity_mps`, B COM hızının gövde eksenlerindeki bileşenleridir. A orijini hızını isteyen adaptör: `uA=uB+Omega*cy`, `vA=vB-Omega*cx`. Yaw hızı tüm gövdede aynıdır. Bu ayrım Gazebo karşılaştırmasında korunmalıdır.

`caster_angle_body_rad=phi`, caster'ın gövdeye göre yuvarlanma yönü; `caster_angle_world_rad=yaw+phi`. Açılar radyan ve süreklidir. `caster_relative_rate_radps=phi_dot`; caster'ın mutlak swivel hızı yaw_dot+phi_dot'tur. Tekerlek roll hızları ileri yuvarlanma yönü pozitif kabul edilir; CAD eklem eksenlerinde gerekirse işaret/ofset adaptörü gerekir.

İntegratör durum sırası: `[Bx,By,yaw,phi,Vx,Vy,yaw_rate,phi_rate,wL,wR,wC,motor_work,dissipated_energy]`. Dünya doğrusal hızları Vx,Vy; birimler m, rad, m/s, rad/s, J. Başlangıç durumlarının tamamı `run_manifest.json` içinde kayıtlıdır. A başlangıç pozu [0,0,0]; B başlangıç konumu body_com_xy'dır.

## Girdi

```csv
time_s,left_drive_torque_Nm,right_drive_torque_Nm
```

Zaman 0'dan başlar, kesin artar. 0,01 s komut örnekleri arasında zero-order hold uygulanır. Torklar tekerlek aksında N·m birimindedir; motor redüksiyonundan önceki tork, voltaj, tekerlek hız hedefi veya gövde hız hedefi değildir. Son satır koşu bitiş zamanıdır. Kontrolcü içermeyen açık çevrim deneyidir.

## Sonuç

44 sayısal sütun:

```csv
time_s,left_drive_torque_Nm,right_drive_torque_Nm,x_m,y_m,yaw_rad,caster_angle_body_rad,caster_angle_world_rad,body_com_x_world_m,body_com_y_world_m,body_long_velocity_mps,body_lat_velocity_mps,body_yaw_rate_radps,caster_relative_rate_radps,left_wheel_rate_radps,right_wheel_rate_radps,caster_roll_rate_radps,left_fx_world_N,left_fy_world_N,right_fx_world_N,right_fy_world_N,caster_fx_world_N,caster_fy_world_N,normal_left_N,normal_right_N,normal_caster_N,left_long_slip_mps,left_lat_slip_mps,right_long_slip_mps,right_lat_slip_mps,caster_long_slip_mps,caster_lat_slip_mps,swivel_bearing_torque_Nm,bore_ground_torque_Nm,caster_external_yaw_moment_Nm,kinetic_energy_J,motor_power_W,dissipation_power_W,motor_work_J,dissipated_energy_J,energy_balance_error_J,linear_momentum_x_kgmps,linear_momentum_y_kgmps,angular_momentum_z_kgm2ps
```

Kuvvetler dünya x/y, normal yükler +Z ve Newton biriminde. Slip sütunları her tekerleğin yuvarlanma/yanal ekseninde m/s'dir. Bearing/bore sütunları işaretli **direnç ifadeleridir**; dinamik denklem bunları ilgili genel kuvvetten çıkarır. `caster_external_yaw_moment_Nm`, B hakkında caster temas kuvveti kaldıraç momenti eksi bore momentidir; tek başına `Ib*yaw_accel` değildir, caster atalet tepkisi de denklemdedir.

Enerji hatası `T-T_initial+D-W`; motor işi negatif olabilir (rejeneratif/aktif frenleme ideal mekanik tork girdisi), D monoton artar. Elektrik güç tüketimi hesabı yapılmaz. Momentumlar dünyanın sabit orijinine göredir; yalnız conservative_free koşusunda korunmaları beklenir.

CSV varsayılan 0,01 s sonuç zamanlarında yazılır; `result_sample_dt_s` ile bağımsız değiştirilebilir. Komut zamanları `command_sample_dt_s` alanından gelir. Sonuçlar solver kayıtlarından örneklenir; CSV tork sütunları sonuç zamanlarına sıfırıncı derece tutmayla taşınır. Varsayılan iç ode4 adımı 0,0005 s, iki yakınsama koşusunda yarısı ve çeyreğidir. İşlenmiş CSV'nin daha seyrek oluşu fizik çözüm adımı değildir. Aynı tork girdisi, başlangıç durumu ve fizik parametreleri olmadan Gazebo–MATLAB farkı kalibrasyon kanıtı sayılmaz.

## Parametre vektörü adaptörü

İnsan tarafından düzenlenecek dosya JSON'dur. Simulink Constant bloğuna taşınan `p_vector` sırası `m2_load_parameters.m` içinde tek yerde tanımlıdır:

```text
1 mb, 2 mc, 3 Ib, 4 Js, 5 Jdrive, 6 JcasterRoll,
7 cx, 8 cy, 9 cz, 10 rdrive, 11 track, 12 rcaster, 13 trail,
14 pivot_x, 15 pivot_y, 16 g, 17 mu_drive, 18 mu_caster,
19/20 drive_k_long/lat, 21/22 caster_k_long/lat,
23/24 rolling_Crr_drive/caster, 25/26 roll_viscous_drive/caster,
27 swivel_viscous, 28 swivel_Coulomb, 29 roll_reg_rate,
30 swivel_reg_rate, 31 bore_effective_radius, 32 bore_reg_rate
```

`cz` kütle merkezi meta verisidir; düşey/yük-transfer dinamiği bu sürümde çözülmediği için planar denkleme girmez. Şasi CAD boyutları ve pivot yüksekliği aynı nedenle geometri sözleşmesinde kalır.

## V1 yürütme kaydı

`run_manifest.json`, `Me492CasterM2V1` kaynak modelini ve izole kopya replay kullanımını belirtir. Her koşu komut/sonuç aralığını, gerçek kullanılan parametre vektörünü ve `commands_md5/states_md5` alanlarını kaydeder. Checksum, Java gerektirmeyen native `Simulink.getFileChecksum` ile MD5 olarak hesaplanır; alan adları algoritmayı açıkça belirtir. Tam `verify_m2`, aynı manifestte 12 senaryo ve değişmemiş CSV'ler ister.

`state_parameter_map.json` 13 durumun birim/çerçevesini ve 32 fizik parametresinin SI birimini içerir. Son iki durum enerji muhasebesidir; 11 fiziksel duruma yeni bir kuvvet geri beslemesi eklemez. Paket üyesi SHA-256 değerleri ayrı `file_manifest_sha256.json` içindedir.
