"""Three-page V2 calculation supplement to the aligned wheel inertias and controller."""
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
TEMP = ROOT / "tmp/pdfs/calculations-only-en"
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



page('1    Wheel inertia and wheel PI')
p('Uniform solid cylinders - central spin axes')
eq(r'm_d=m_c=0.4\ \mathrm{kg},\quad r_d=0.075\ \mathrm m,\quad r_c=0.040\ \mathrm m')
eq(r'\sigma=\frac{m}{\pi r^2},\qquad dm=2\pi\sigma\rho\,d\rho')
eq(r'J=\int_0^r\rho^2\,dm=2\pi\sigma\int_0^r\rho^3\,d\rho=\frac{\pi\sigma r^4}{2}=\frac{mr^2}{2}')
eq(r'J_d=\frac{0.4(0.075)^2}{2}=0.001125\ \mathrm{kg\,m^2}')
eq(r'J_c=\frac{0.4(0.040)^2}{2}=0.000320\ \mathrm{kg\,m^2}')
p('Isolated wheel - fixed axis, no contact or losses')
eq(r'J_d\dot\Omega=\tau,\quad\tau=0.01\ \mathrm{N\,m},\quad\Omega(0)=0')
eq(r'\dot\Omega=\frac{0.01}{0.001125}=8.888888889\ \mathrm{rad/s^2},\quad\Omega(0.2)=1.777777778\ \mathrm{rad/s}')
p('PI - constant load, constant reference, no saturation')
eq(r'J\dot\Omega=\tau-d,\quad e=\Omega^*-\Omega,\quad\tau=K_pe+I,\quad\dot I=K_ie')
eq(r'K_p=0.15\ \mathrm{N\,m\,s/rad},\quad K_i=0.8\ \mathrm{N\,m/rad},\quad\widetilde I=I-d')
eq(r'J\dot e=-K_pe-\widetilde I,\qquad\dot{\widetilde I}=K_ie')
eq(r'V_i=\frac{Je^2}{2}+\frac{\widetilde I^2}{2K_i}')
eq(r'\dot V_i=Je\dot e+\frac{\widetilde I\dot{\widetilde I}}{K_i}=e(-K_pe-\widetilde I)+\widetilde I e=-K_pe^2\leq0')
eq(r'\dot V_i=0\Rightarrow e=0,\quad\dot e=0\Rightarrow\widetilde I=0\quad\Rightarrow\quad e\to0,\ I\to d')
p('Sampled PI - integral updated before torque')
eq(r'h=0.01\ \mathrm s,\quad I_k=I_{k-1}+K_ihe_k,\quad\tau_k=K_pe_k+I_k')
eq(r'e_{k+1}=(1-a-b_i)e_k-\frac{h}{J_d}\widetilde I_{k-1},\qquad\widetilde I_k=\widetilde I_{k-1}+K_ihe_k')
eq(r'a=\frac{hK_p}{J_d}=1.333333,\quad b_i=\frac{h^2K_i}{J_d}=0.071111')
eq(r'\mathrm{tr}(A)=2-a-b_i,\quad\det A=1-a,\quad a>0,\ b_i>0,\ 4-2a-b_i=1.262222>0')
eq(r'\lambda_1=-0.351841,\quad\lambda_2=0.947397,\qquad |\lambda_1|,|\lambda_2|<1')

page('2    Tracking control')
p('Ideal axle kinematics - continuous angle branch, no saturation, v=v_c and ω=ω_c')
eq(r'e(\theta)=(\cos\theta,\sin\theta)^T,\quad n(\theta)=(-\sin\theta,\cos\theta)^T')
eq(r'R(\theta)=[\ e(\theta)\ \ n(\theta)\ ],\quad E=\left[\ (0,1)^T\ \ (-1,0)^T\ \right],\quad\dot R^T=-\omega ER^T')
eq(r'p_A=p_B-R(\theta)c,\qquad\dot p_A=v(\cos\theta,\sin\theta)^T,\quad\dot\theta=\omega')
eq(r'(e_x,e_y)^T=R(\theta)^T(p_r-p_A),\qquad e_\theta=\theta_r-\theta')
eq(r'\dot e_x=\omega e_y-v+v_r\cos e_\theta,\qquad\dot e_y=-\omega e_x+v_r\sin e_\theta')
eq(r'\dot e_\theta=\omega_r-\omega,\quad v_c=v_r\cos e_\theta+k_xe_x,\quad\omega_c=\omega_r+k_yv_re_y+k_\theta\sin e_\theta')
eq(r'k_x=2\ \mathrm{s^{-1}},\qquad k_y=6\ \mathrm{m^{-2}},\qquad k_\theta=3\ \mathrm{s^{-1}}')
eq(r'V=\frac{e_x^2+e_y^2}{2}+\frac{1-\cos e_\theta}{k_y}')
eq(r'\dot V=e_x(\omega e_y-v+v_r\cos e_\theta)+e_y(-\omega e_x+v_r\sin e_\theta)+\frac{\sin e_\theta}{k_y}(\omega_r-\omega)')
eq(r'\dot V=-k_xe_x^2+v_re_y\sin e_\theta-\frac{\sin e_\theta}{k_y}(k_yv_re_y+k_\theta\sin e_\theta)')
eq(r'\dot V=-k_xe_x^2-\frac{k_\theta}{k_y}\sin^2e_\theta=-2e_x^2-\frac12\sin^2e_\theta\leq0')
p('Numerical substitution')
eq(r'e_x=0.01\ \mathrm m,\quad e_y=-0.005\ \mathrm m,\quad e_\theta=0.1\ \mathrm{rad}')
eq(r'v_r=0.15\ \mathrm{m/s},\quad\omega_r=\pi/6\ \mathrm{rad/s}')
eq(r'v_c=0.15\cos(0.1)+2(0.01)=0.169250625\ \mathrm{m/s}')
eq(r'\omega_c=\pi/6+6(0.15)(-0.005)+3\sin(0.1)=0.818599026\ \mathrm{rad/s}')
eq(r'\dot V=-2(0.01)^2-\frac12\sin^2(0.1)=-0.005183356\ \mathrm{m^2/s}')
eq(r'\Omega_L^*=\frac{v_c-b\omega_c/2}{r_d},\qquad\Omega_R^*=\frac{v_c+b\omega_c/2}{r_d},\qquad b=0.38\ \mathrm m')

page('3    Terminal position and heading')
p('Ideal axle kinematics - no saturation; fixed goal; ρ>0; s fixed during approach')
eq(r'\delta=p_g-p_A,\quad\rho=\|\delta\|,\quad\beta=\mathrm{atan2}(\delta_y,\delta_x)')
eq(r's\in\{-1,+1\},\quad\theta_e=\theta+\frac{1-s}{2}\pi,\quad\alpha=\mathrm{wrap}(\beta-\theta_e),\quad|\alpha_0|\leq\pi/2')
eq(r'u=sv,\qquad\dot\rho=-u\cos\alpha,\quad\dot\beta=\frac u\rho\sin\alpha,\quad\dot\alpha=\frac u\rho\sin\alpha-\omega')
eq(r'v_c=s k_\rho\rho\cos\alpha,\quad\omega_c=k_\alpha\alpha+k_\rho\sin\alpha\cos\alpha,\quad k_\rho=2,\ k_\alpha=3')
eq(r'\dot\rho=-k_\rho\rho\cos^2\alpha=-2\rho\cos^2\alpha')
eq(r'\dot\alpha=k_\rho\cos\alpha\sin\alpha-(k_\alpha\alpha+k_\rho\sin\alpha\cos\alpha)=-3\alpha')
eq(r'\alpha(t)=\alpha_0e^{-3t},\qquad\rho(t)=\rho_0\exp\!\left[-2\int_0^t\cos^2\alpha(\xi)\,d\xi\right]')
eq(r't_1=\frac{1}{3}\ln\!\left[\max\left(1,\frac{4|\alpha_0|}{\pi}\right)\right]')
eq(r't\geq t_1\Rightarrow|\alpha(t)|\leq\pi/4\Rightarrow\dot\rho\leq-\rho')
eq(r'0\leq\rho(t)\leq\rho(t_1)e^{-(t-t_1)}\longrightarrow0')
p('Numerical substitution')
eq(r'\delta=(0.030,0.001)^T\ \mathrm m,\quad\theta=0,\quad s=+1')
eq(r'\rho=\sqrt{0.030^2+0.001^2}=0.030016662\ \mathrm m,\quad\alpha=\arctan(1/30)=0.033320996\ \mathrm{rad}')
eq(r'v_c=2\rho\cos\alpha=0.060000000\ \mathrm{m/s},\quad\omega_c=3\alpha+2\sin\alpha\cos\alpha=0.166555662\ \mathrm{rad/s}')
p('Final heading - ideal in-place rotation, |e_g|<π')
eq(r'e_g=\mathrm{wrap}(\theta_g-\theta),\quad v_c=0,\quad\omega_c=3\sin e_g,\quad\dot e_g=-3\sin e_g')
eq(r'V_g=1-\cos e_g,\qquad\dot V_g=\sin e_g\dot e_g=-3\sin^2e_g\leq0')
eq(r'\dot V_g=0,\ |e_g|<\pi\Rightarrow e_g=0\quad\Rightarrow\quad e_g(t)\to0')

expressions=[s.replace(r'\frac12',r'\frac{1}{2}').replace(r'\frac u\rho',r'\frac{u}{\rho}').replace(r'\mathrm m',r'\mathrm{m}').replace(r'\mathrm s',r'\mathrm{s}') for s in expressions]
(TEMP/'equations.json').write_text(json.dumps(expressions,ensure_ascii=False))
subprocess.run([str(ROOT.parent/'caster-physics-python/.venv/bin/python'),str(ROOT/'docs/render_math_assets.py'),str(TEMP/'equations.json'),str(TEMP)],check=True)
def footer(canvas,document):
    canvas.setFont('Body',8);canvas.setFillColor(colors.HexColor('#566674'))
    canvas.drawRightString(543,26,f'{document.page} / 3')
destination=ROOT/'output/pdf/Arche_Calculation_Proofs_EN.pdf'
doc=SimpleDocTemplate(str(destination),pagesize=(595.28,841.89),leftMargin=52,rightMargin=52,topMargin=40,bottomMargin=44,title='Calculation proofs',author='')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
assert len(PdfReader(destination).pages)==3
print(destination)
