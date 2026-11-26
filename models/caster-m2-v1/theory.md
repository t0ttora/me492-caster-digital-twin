# M2 denklemleri ve enerji tutarlılığı

## Konum, serbestlik dereceleri, atalet

Kanonik robot orijini A: tahrik aksı orta noktasının zemine izdüşümü. +X ileri, +Y sol, +Z yukarı. Dinamik öteleme koordinatı **B gövde kütle merkezinin dünya konumudur**. Sabit gövde COM ofseti `c=[−0,08;0] m`; `B=A+R(theta)c`. Caster pivotu H: `A+R(theta)h`, `h=[−0,18;0] m`. Caster aks merkezi C: `H-l e(psi)`, trail `l=0,05 m`, `psi=theta+phi`.

Temel koordinatlar `q=[Bx,By,theta,phi,alphaL,alphaR,alphaC]`. Üç tekerlek roll açısı kuvvet hesabında döngüsel olduğundan yalnız hızları saklanır. Genel hız:

```text
nu = [Vx, Vy, Omega, rho, wL, wR, wC]
psi_dot = Omega + rho; phi_dot = rho
```

13 Simulink durumu: `Bx,By,theta,phi`, bu yedi hız, birikmiş motor işi W ve birikmiş kayıp D. Swivel ataleti mutlak `psi_dot` ile enerji taşır. Gövde yaw ataleti `Ib` B çevresindedir; caster'ın trail ötelenme ataleti ayrıca hesaplanır. Caster nokta kütlesi `mc` C'de, kendi COM çevresindeki yaw ataleti `Js` ile temsil edilir.

`E=[[0,-1],[1,0]]` düzlemsel 90° döndürme matrisi olsun. B'den pivot vektörü `rH=R(theta)(h-c)`, caster vektörü `rC=rH-l e(psi)`. Caster Cartesian hız Jacobian'ı:

```text
JC = [I2, E*rC, -l*n(psi), 0, 0, 0]
VC = JC*nu
bC = -rH*Omega^2 + l*e(psi)*(Omega+rho)^2
```

Tahrik merkezleri A'ya göre `[0,+Btrack/2]`, `[0,-Btrack/2]`; `JL/JR=[I2,E*rL/R,0,0,0,0]`. Jacobian'lar 2×7'dir. Rotor yönlerinin değişiminden doğan düzlem dışı jiroskopik momentler, sabit roll/pitch kısıtları içinde bırakılır.

Kinetik enerji:

```text
T = mb/2 * |VB|^2 + Ib/2 * Omega^2
  + mc/2 * |VC|^2 + Js/2 * (Omega+rho)^2
  + Jd/2 * (wL^2+wR^2) + Jc/2 * wC^2
```

`a=[0,0,1,1,0,0,0]'` ile:

```text
M = diag(mb,mb,Ib,0,Jd,Jd,Jc) + mc*JC'*JC + Js*a*a'
bias = mc*JC'*bC
M*nu_dot + bias = Q
```

Bu, nokta kütlesi için Newton kuvvet dengesinin Jacobian ile genel koordinatlara taşınmasıyla veya aynı enerjinin Lagrange türeviyle elde edilir. `Js>0` ve diğer atalet/kütleler pozitif olduğunda M pozitif tanımlıdır. Caster kuvvetini gövdeye tek yönlü ekleyip swivel ataleti tepkisini ihmal etmiyoruz.

## Kayma, kuvvet ve moment

Tekerleğin yuvarlanma yönü `e`, yanal yönü `n`. Kayma hızları:

```text
s_long = e'*(J*nu) - radius*w
s_lat  = n'*(J*nu)
```

Tahrik için `z=[kx*s_long,ky*s_lat]`, `L=mu*N`. Kuvvet `f=-z*L/sqrt(L^2+z'z)`; L=0 ise f=0. Böylece `|f|<=mu*N` ve `f' s<=0`. Bu bir boyutsuz creepage/Hertz hesabı değil, düşük hızda da tanımlı kayma-hızı direnç modelidir.

Caster için bore momenti aynı kapasiteyi paylaşır. Etkin patch yarıçapı `rb`, hız ölçeği `ws`:

```text
z = [kx*s_long, ky*s_lat, L*psi_dot/ws]
scale = L / sqrt(L^2+z'z)
fC_local = -scale*z(1:2)
tau_bore = scale*z(3)*rb
sqrt(|fC|^2 + (tau_bore/rb)^2) <= mu*N
```

`rb=0` ise üçüncü bileşen sıfırdır. Bu ortak elipsoidal sınır fenomenolojiktir; gerçek contact patch'in tam kuvvet/moment akma yüzeyi olarak yorumlanmaz. Kuvvet dünyaya tekerlek yönleri ile çevrilir.

Temasın kayma hızını veren dünya Jacobian'ları `GL=JL`'nin wL sütununa `−rd*eD`, `GR`'nin wR sütununa `−rd*eD`, `GC`'nin wC sütununa `−rc*eC` koyularak elde edilir. Böylece:

```text
Q_contact = GL'*FL + GR'*FR + GC'*FC
```

Bu ifade gövde kuvvetini, kaldıraç yaw momentini, caster swivel momentini ve roll temas reaksiyonunu **aynı kuvvetten** üretir. Örneğin tahrik tekerleği `Jd*w_dot = tau_drive - rd*f_long - tau_roll` işaretini taşır.

Rolling direnç: `tau_roll=Crr*N*r*tanh(w/wreg)+broll*w`. Bearing direnç: `tau_bearing=bs*rho+tau_s*tanh(rho/rhoreg)`. Bearing, relative phi üzerinde `Qphi-=tau_bearing`; iç moment olduğu için doğrudan Qtheta'ya eklenmez. Tepki bağlı atalet matrisinden gelir. Bore zemin momenti mutlak psi üzerinde olduğu için **Qtheta ve Qphi'den birlikte çıkarılır**.

## Destek yükleri ve COM

Gövde düzlemi sabit yükseklikte tutulur. Anlık caster konumunun A gövde koordinatları `Cbody=[a-l cos(phi), b-l sin(phi)]`; toplam COM `ctotal=(mb*c+mc*Cbody)/(mb+mc)`. Statik üçgen dengesi:

```text
sum(Ni) = (mb+mc)*g
sum(Ni*contact_xi) = g*(mb*cx + mc*Cbody_x)
sum(Ni*contact_yi) = g*(mb*cy + mc*Cbody_y)
```

Bu üç denklem Nl,Nr,Nc'yi belirler. Negatif yük kırpılmaz; destek üçgeninin dışına çıkan COM parametresi reddedilir. Mevcut COM tüm swivel açılarında içerdedir. Düşey ivme ve pitch/roll dinamiği olmadığından ivmeye bağlı normal yük transferi yoktur; büyük ivme, darbe veya devrilme araştırması için model genişletilmelidir.

## Enerji ve bağımsız kontroller

Motor gücü `Pin=tauL*wL+tauR*wR`; kayıp:

```text
Pdiss = -FL'*(GL*nu) - FR'*(GR*nu) - FC'*(GC*nu)
      + tau_rollL*wL + tau_rollR*wR + tau_rollC*wC
      + tau_bearing*rho + tau_bore*(Omega+rho) >= 0
W_dot=Pin; D_dot=Pdiss; T - T_initial + D - W = 0
```

Testler T'yi M matrisini kullanmadan doğrudan nokta hızlarıyla yeniden hesaplar ve merkezî sonlu farkla dT/dt'yi Pin−Pdiss ile karşılaştırır. Simulink çıktısında entegre enerji dengesi, pasif enerji azalması, dirençsiz durumda enerji ve dünya doğrusal/açısal momentum korunumu denetlenir. Ayna koşusu kuvvet/yaw işaretini; h/h2/h4 yakınsaması sayısal etkinin büyüklüğünü sınar. Caster etkisi için önceden tanımlı eşik 1 mm ve 0,001 rad; ayrıca adım farkının en az 100 katı aranır.

## Birincil kaynaklar ve ayrım

[Schröder, Lenord, Lange (2019), Enhanced Motion Control of a Self-Driving Vehicle Using Modelica, FMI and ROS](https://2019.international.conference.modelica.org/proceedings/html/papers/Modelica2019paper4B2.pdf), bölüm 2, düzlemsel tahrik/caster yapısı, statik yük yaklaşımı ve bore direncinin önemine temel oluşturur. Çalışma kilitlenme için hibrit stiction modeli kullanır; bizim yumuşak modelimiz bu davranışı birebir çözmez. Ters sürüşte anında zorunlu caster dönüşü kabul etmememiz de bu ayrımla tutarlıdır.

[Jaswin, Indrawanto (2020), Application of Kalker’s Theory of Rolling Contact for Dynamic Simulation of Mobile Robots](https://arxiv.org/pdf/2011.01066), bölüm III, kayma-kuvvet ilişkisi, Newton–Euler gövde dengesi ve `J*w_dot=tau-r*f` işaretine kaynak sağlar. Biz Kalker/FASTSIM uygulamadık; kendi regularized kuvvet modelimizin sentetik katsayılarını kullandık. Yukarıdaki bağlı caster kütle matrisi bu paketin açık geometri varsayımlarından türetilmiştir.

Simulink'te kod ile sürekli integratör bağlantısı için [MathWorks MATLAB Function](https://www.mathworks.com/help/simulink/slref/matlabfunction.html) ve [Integrator](https://www.mathworks.com/help/simulink/slref/integrator.html) belgeleri esas alındı.
