"""Three-page V2 calculation supplement to the user's existing V1 derivation."""
import json
from pathlib import Path
import subprocess
from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, PageBreak, Table, TableStyle, Flowable
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
TEMP = ROOT / "tmp/pdfs/short-calculations"
TEMP.mkdir(parents=True, exist_ok=True)
FONT = Path('/Users/tora/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/poppler/fonts')
for name,file in [('Body','DejaVuSans.ttf'),('Bold','DejaVuSans-Bold.ttf')]:
    path=FONT/file
    pdfmetrics.registerFont(TTFont(name,str(path if path.exists() else FONT/'DejaVuSans.ttf')))
body=ParagraphStyle('body',fontName='Body',fontSize=9.3,leading=13.2,spaceAfter=6)
heading=ParagraphStyle('heading',fontName='Bold',fontSize=16,leading=20,spaceAfter=9)
small=ParagraphStyle('small',fontName='Body',fontSize=8,leading=11,spaceAfter=5)
story,expressions=[],[]

class Eq(Flowable):
    def __init__(self,text):
        super().__init__();self.index=len(expressions);expressions.append(text)
    def wrap(self,width,height):
        self.path=TEMP/f'eq-{self.index:03d}.png'
        with PILImage.open(self.path) as im:iw,ih=im.size
        scale=min(72/240,(width-6)/iw)
        self.iw,self.ih=iw*scale,ih*scale
        self.width,self.height=width,self.ih+12
        return self.width,self.height
    def draw(self):
        self.canv.drawImage(str(self.path),2,6,width=self.iw,height=self.ih,mask='auto')

def p(text,tiny=False):story.append(Paragraph(text,small if tiny else body))
def eq(text):story.append(Eq(text))
def page(title):
    if story:story.append(PageBreak())
    story.append(Paragraph(title,heading))

page('1 / 3    Takip kontrolünün hesabı')
p('V1’de türettiğin mekanik denklem korunur. Senin notasyonunla c=AB, h=AH, E=[0 −1; 1 0] ve p_A=p_B−R(θ)c. V2 yalnız motor torkunun nasıl seçildiğini değiştirir. Aşağıda θ gövde açısı, ω=θ̇ gövde yaw hızı; Ω_L,Ω_R tekerlek hızlarıdır.')
eq(r'M(q)\dot\nu+b(q,\nu)=Q(q,\nu,\tau_L,\tau_R)')
p('Hedef aks pozu (p_r,θ_r), referans hızları v_r,ω_r. Hataları robot eksenlerinde yaz:')
eq(r'(e_x,e_y)^T=R(\theta)^T(p_r-p_A),\qquad e_\theta=\theta_r-\theta')
p('İdeal aks kinematiği ṗ_A=v(cosθ,sinθ)ᵀ, θ̇=ω ve Ṙᵀ=−ωERᵀ ile türev al:')
eq(r'\dot e_x=\omega e_y-v+v_r\cos e_\theta,\quad\dot e_y=-\omega e_x+v_r\sin e_\theta')
eq(r'\dot e_\theta=\omega_r-\omega,\qquad v_c=v_r\cos e_\theta+2e_x,\quad\omega_c=\omega_r+6v_re_y+3\sin e_\theta')
p('Doygunluk ve servo/kayma hatası yokken v=v_c, ω=ω_c koy. Lyapunov fonksiyonunu türet; çapraz terimler iptal olur:')
eq(r'V=\frac12(e_x^2+e_y^2)+\frac{1-\cos e_\theta}{6}')
eq(r'\dot V=e_x(\omega e_y-v+v_r\cos e_\theta)+e_y(-\omega e_x+v_r\sin e_\theta)+\frac{\sin e_\theta}{6}(\omega_r-\omega)')
eq(r'\dot V=-2e_x^2+v_re_y\sin e_\theta-\frac{\sin e_\theta}{6}(6v_re_y+3\sin e_\theta)')
eq(r'\boxed{\dot V=-2e_x^2-\frac12\sin^2e_\theta\leq0}')
p('Elle sayısal kontrol: e_x=0.01 m, e_y=−0.005 m, e_θ=0.1 rad; v_r=0.15 m/s, ω_r=π/6 rad/s.')
eq(r'v_c=0.15\cos(0.1)+2(0.01)=0.169250625\ \mathrm{m/s}')
eq(r'\omega_c=\pi/6+6(0.15)(-0.005)+3\sin(0.1)=0.818599026\ \mathrm{rad/s}')
eq(r'\dot V=-2(0.01)^2-\frac12\sin^2(0.1)=-0.005183356\ \mathrm{m^2/s}')
p('Önemli sonuç: V̇≤0 tek başına e_y→0 ispatı değildir. v_r=0 iken yanal düzeltme kaybolur; bu yüzden duruş hedefi için 3. sayfadaki kontrol gerekir. Kazançların birimleri sırasıyla 2 s⁻¹, 6 m⁻², 3 s⁻¹’dir.',True)

page('2 / 3    Tekerlek PI kontrolünün hesabı')
p('b=0.38 m, r_d=0.075 m. Gövde komutlarını tekerlek referansına çevir:')
eq(r'\Omega_L^*=\frac{v_c-b\omega_c/2}{r_d},\qquad\Omega_R^*=\frac{v_c+b\omega_c/2}{r_d}')
p('Her teker için e=Ω*−Ω, K_p=0.15 N m s/rad, K_i=0.8 N m/rad, Δt=0.01 s. Saklanan I bir torktur. Kodun güncelleme sırası:')
eq(r'I_{\rm cand}=I_{k-1}+0.8\Delta t\,e_k,\quad\tau_{\rm raw}=0.15e_k+I_{\rm cand},\quad\tau_s=\mathrm{clip}(\tau_{\rm raw},-0.5,0.5)')
p('|τ_raw|≤0.5 veya e_k(τ_raw−τ_s)≤0 ise I_k=I_cand; aksi halde I_k=I_(k−1). Sonra τ_k=clip(0.15e_k+I_k,±0.5). Böylece doygunluğu artıran hata integrale eklenmez; tork her zaman sınırlıdır.')
p('Sabit yük için kısa ispat: yalnız skaler alt model JΩ̇=τ−d, sabit Ω*, sabit d ve doygunluk yok kabul et. Ĩ=I−d yaz:')
eq(r'J\dot e=-K_pe-\widetilde I,\quad\dot{\widetilde I}=K_ie,\quad V_i=\frac12Je^2+\frac{\widetilde I^2}{2K_i}')
eq(r'\dot V_i=e(-K_pe-\widetilde I)+\widetilde I e=-K_pe^2\leq0')
p('V̇_i=0 kümesinde e=0; orada kalmak için ė=−Ĩ/J=0, dolayısıyla Ĩ=0 gerekir. Böylece sabit yükte e→0, I→d. Eski P kontrolünde ise denge hatası e*=d/K_p idi.')
p('Elle iki adım: J=0.0045 kg m², d=(0.08,0.10) N m, Ω*=(2,2) rad/s, Ω₀=I₋₁=0. Aralıkta sabit torkla Ω_(k+1)=Ω_k+Δt(τ_k−d)/J.')
rows=[['k','Ω_k [rad/s]','I_k [N m]','τ_k [N m]','Ω_(k+1) [rad/s]'],
      ['0','(0, 0)','(.016, .016)','(.316, .316)','(.524444, .480000)'],
      ['1','(.524444, .480000)','(.027804, .028160)','(.249138, .256160)','(.900306, .827022)']]
tab=Table([[Paragraph(c,small) for c in row] for row in rows],colWidths=[23,115,109,109,125])
tab.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#edf1f4')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),6)]))
story.append(tab)
p('Aynı hesap 400 adımda Ω→(2,2), I→(0.08,0.10) verir; hız hatası 10⁻⁵ rad/s altındadır. Eski P denge hatası (0.533333,0.666667) rad/s olur.')
p('Örneklemeli doğruluk kontrolü: a=ΔtK_p/J=1/3, b_i=Δt²K_i/J=0.017778. Yukarıdaki güncellemenin tr(A)=2−a−b_i, det(A)=1−a; Jury koşulları a>0, b_i>0, 4−2a−b_i=3.315556>0. Özdeğerler 0.710243 ve 0.938646; ikisi de birim çember içinde.',True)
p('Atalet ayrımı: bu skaler hesap Python V1’deki J_d=0.0045 ile yapıldı. Ekindeki izole Simscape teker deneyinde J=½mr²=0.001125 kg m² kullanılmış; bu iki örnek aynı parametreli deney değildir. Tam M2, skaler sabit-yük alt modelinden daha kapsamlıdır.',True)

page('3 / 3    Hedefte konum ve açı düzeltmesi')
p('Profil bitince hedef (p_g,θ_g) sabittir. ρ=‖p_g−p_A‖, β=atan2(y_g−y_A,x_g−x_A). İlk girişte ileri/geri yön s∈{+1,−1}, |α₀|≤π/2 olacak şekilde seçilir; yaklaşma boyunca sabit tutulur:')
eq(r'\theta_e=\theta+\frac{1-s}{2}\pi,\qquad\alpha=\mathrm{wrap}(\beta-\theta_e)')
eq(r'v_c=s\,2\rho\cos\alpha,\qquad\omega_c=3\alpha+2\sin\alpha\cos\alpha')
p('İdeal ve doygunluksuz aks için u=s v; ρ>0 bölgesinde geometrinin türevi:')
eq(r'\dot\rho=-u\cos\alpha,\qquad\dot\alpha=\frac u\rho\sin\alpha-\omega')
eq(r'\dot\rho=-2\rho\cos^2\alpha,\qquad\dot\alpha=-3\alpha\quad\Rightarrow\quad\alpha(t)=\alpha_0e^{-3t}')
eq(r'\rho(t)=\rho_0\exp\!\left[-2\int_0^t\cos^2\alpha(\xi)\,d\xi\right]\longrightarrow0')
p('Gerekçe: sonlu t₁’den sonra |α|≤π/4, yani cos²α≥1/2 olur; bundan sonra ρ̇≤−ρ ve ρ(t)≤ρ(t₁)e⁻⁽ᵗ⁻ᵗ¹⁾. Bu, nominal konum yakınsamasını gösterir; hedef yaw henüz düzenlenmiş değildir.')
p('Elle sayısal kontrol: p_g−p_A=(0.030,0.001) m, θ=0, s=+1.')
eq(r'\rho=0.030016662\ \mathrm m,\quad\alpha=\arctan(1/30)=0.033320996\ \mathrm{rad}')
eq(r'v_c=0.060000000\ \mathrm{m/s},\quad\omega_c=0.166555662\ \mathrm{rad/s}')
p('ρ≤6 mm olunca son açıya geç: e_g=wrap(θ_g−θ), v_c=0, ω_c=3 sin(e_g).')
eq(r'V_g=1-\cos e_g,\qquad\dot V_g=-3\sin^2e_g\leq0\qquad(|e_g|<\pi)')
p('İdeal yerinde dönmede konum sabit kalır ve hedef yaw’a yerel yakınsama vardır. M2’de kayma konumu bozabileceğinden ρ>12 mm olursa konum yaklaşmasına dönülür. Tüm modlarda |v_c|≤0.25 m/s, |ω_c|≤1 rad/s, |τ|≤0.5 N m uygulanır.')
p('Bitiş kontrolü: konum≤20 mm, |yaw hatası|≤0.05 rad, toplam düzlemsel hız≤0.02 m/s ve |yaw hızı|≤0.05 rad/s; hepsi 0.25 s korunmalı. Aynı 1 s pencerede 3/16→8/16; açık 8 s terminal gözlemde 16/16, caster taramasıyla 40/40 tamamlandı. Bunlar sayısal sonuçtur.',True)
p('İspat sınırı: burada ideal alt modellerin cebirsel sonuçları gösterildi. Doygunluk, örnekleme, kayma ve mod geçişlerini birlikte içeren tam M2 için küresel kararlılık ispatı değildir. V1 mekanik hesabın tekrarlanmadı; kod karşılığı twin.py / TrackingController’dır.',True)

# Mathtext uses a subset of TeX; remove only the display box, preserving its contents.
expressions=[s[7:-1] if s.startswith(r'\boxed{') and s.endswith('}') else s for s in expressions]
expressions=[s.replace(r'\frac12',r'\frac{1}{2}').replace(r'\frac u\rho',r'\frac{u}{\rho}').replace(r'\mathrm m',r'\mathrm{m}') for s in expressions]
(TEMP/'equations.json').write_text(json.dumps(expressions,ensure_ascii=False))
subprocess.run([str(ROOT.parent/'caster-physics-python/.venv/bin/python'),str(ROOT/'docs/render_math_assets.py'),str(TEMP/'equations.json'),str(TEMP)],check=True)

def footer(canvas,document):
    canvas.setFont('Body',7.5);canvas.setFillColor(colors.HexColor('#566674'))
    canvas.drawString(52,26,'Arche V2.1 | V1 hesabına ek | 13 Kasım 2026')
    canvas.drawRightString(543,26,f'{document.page} / 3')

destination=ROOT/'output/pdf/Arche_V2_Kisa_Hesap_TR.pdf'
doc=SimpleDocTemplate(str(destination),pagesize=(595.28,841.89),leftMargin=52,rightMargin=52,topMargin=44,bottomMargin=46,
                      title='Arche V2 kısa hesap',author='Arche ME492')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
reader=PdfReader(destination)
assert len(reader.pages)==3,len(reader.pages)
assert all(len(p.extract_text())>300 for p in reader.pages)
print(destination)
