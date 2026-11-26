"""Three-page V2 calculation supplement to the aligned wheel inertias and controller."""
import json
from pathlib import Path
import subprocess
import re
from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, PageBreak, Table, TableStyle, Flowable
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
TEMP = ROOT / "tmp/pdfs/v2-controller-design-en"
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
    def __init__(self,text,source=None):
        super().__init__();self.index=len(expressions);expressions.append(text);self.source=source
    def wrap(self,width,height):
        self.path=TEMP/f'eq-{self.index:03d}.png'
        with PILImage.open(self.path) as im:iw,ih=im.size
        scale=min(72/240,(width-6)/iw)
        self.iw,self.ih=iw*scale,ih*scale
        self.width,self.height=width,self.ih+12+(15 if self.source else 0)
        return self.width,self.height
    def draw(self):
        offset=15 if self.source else 0
        self.canv.drawImage(str(self.path),2,6+offset,width=self.iw,height=self.ih,mask='auto')
        if self.source:
            c=self.canv
            c.setStrokeColor(colors.HexColor('#45647a'));c.setFillColor(colors.HexColor('#45647a'))
            c.setLineWidth(0.6)
            c.line(8,offset+4,8,5);c.line(8,5,25,5)
            c.line(25,5,21,7);c.line(25,5,21,3)
            c.setFont('Body',7.1);c.drawString(30,2,self.source)

class MathParagraph(Flowable):
    def __init__(self, text, style):
        super().__init__()
        self.style = style
        self.parts = re.split(r'(\$[^$]+\$)', text)
        self.indices = {}
        for index, part in enumerate(self.parts):
            if part.startswith('$'):
                self.indices[index] = len(expressions)
                expressions.append(part[1:-1])
    def wrap(self, width, height):
        parts = self.parts.copy()
        for index, asset in self.indices.items():
            path = TEMP/f'eq-{asset:03d}.png'
            with PILImage.open(path) as image:
                iw, ih = image.size
            scale = (72/240)*(self.style.fontSize/13)
            parts[index] = f'<img src="{path}" width="{iw*scale}" height="{ih*scale}" valign="-2"/>'
        self.paragraph = Paragraph(''.join(parts), self.style)
        self.width, self.height = self.paragraph.wrap(width, height)
        return self.width, self.height
    def draw(self):
        self.paragraph.drawOn(self.canv, 0, 0)

def p(text,tiny=False):
    paragraph = MathParagraph(text, small if tiny else body)
    paragraph.spaceAfter = (small if tiny else body).spaceAfter
    story.append(paragraph)
def eq(text,source=None):story.append(Eq(text,source))
def page(title):
    if story:story.append(PageBreak())
    story.append(Paragraph(title,heading))



page('1    V2 tracking law and calculated gains')
p(r'Design specifications selected for this calculation: $\tau_x=0.5\,\mathrm{s}$; lateral damping ratio $\zeta=1$ and natural frequency $\omega_n=1\,\mathrm{rad/s}$. These are design choices, not measured robot data.')
eq(r'v_0=\frac{0.30\ \mathrm m}{2\ \mathrm{s}}=0.15\ \mathrm{m/s}')
p(r'Nominal axle kinematics: no lateral slip; $v=v_c$, $\omega=\omega_c$; continuous angle branch.')
eq(r'(e_x,e_y)^T=R(\theta)^T(p_r-p_A),\qquad e_\theta=\theta_r-\theta')
eq(r'\dot e_x=\omega e_y-v+v_r\cos e_\theta,\quad\dot e_y=-\omega e_x+v_r\sin e_\theta,\quad\dot e_\theta=\omega_r-\omega','Kanayama et al., IROS 1991, Eq. (7).')
eq(r'v_c=v_r\cos e_\theta+k_xe_x,\qquad\omega_c=\omega_r+k_yv_re_y+k_\theta\sin e_\theta','Adapted from Kanayama et al., IROS 1991, Eq. (8).')
eq(r'V=\frac12(e_x^2+e_y^2)+\frac{1-\cos e_\theta}{k_y}','Kanayama et al., IROS 1991, Eq. (10).')
eq(r'\dot V=e_x(\omega e_y-v+v_r\cos e_\theta)+e_y(-\omega e_x+v_r\sin e_\theta)+\frac{\sin e_\theta}{k_y}(\omega_r-\omega)')
eq(r'\dot V=-k_xe_x^2+v_re_y\sin e_\theta-\frac{\sin e_\theta}{k_y}(k_yv_re_y+k_\theta\sin e_\theta)')
eq(r'\dot V=-k_xe_x^2-\frac{k_\theta}{k_y}\sin^2e_\theta\leq0','V2 derivation: differentiate and substitute the modified law.')
p(r'Gain calculation: linearize about a straight reference $v_r=v_0$, $\omega_r=0$, $e=0$.')
eq(r'\dot e_x\simeq-k_xe_x,\quad\dot e_y\simeq v_0e_\theta,\quad\dot e_\theta\simeq-k_yv_0e_y-k_\theta e_\theta')
eq(r'\ddot e_y+k_\theta\dot e_y+k_yv_0^2e_y=0')
eq(r's^2+k_\theta s+k_yv_0^2=s^2+2\zeta\omega_ns+\omega_n^2=(s+1)^2','Åström & Murray, Feedback Systems (2nd ed., 2020), Eq. (7.22), p. 7-17.')
eq(r'k_x=\frac1{\tau_x}=\frac1{0.5}=2\ \mathrm{s^{-1}}','Åström & Murray, Feedback Systems (2020), Exercise 6.7, p. 6-36.')
eq(r'k_\theta=2\zeta\omega_n=2\ \mathrm{s^{-1}},\qquad k_y=\frac{\omega_n^2}{v_0^2}=\frac1{0.15^2}=44.444444\ \mathrm{m^{-2}}','V2 coefficient matching to Feedback Systems, Eq. (7.22).')
eq(r'v_c=v_r\cos e_\theta+2e_x,\quad\omega_c=\omega_r+44.444444v_re_y+2\sin e_\theta')
eq(r'\dot V=-2e_x^2-0.045\sin^2e_\theta\leq0')
p(r'The pole placement is local to a nonzero nominal speed. $\dot V\leq0$ alone does not establish lateral convergence at $v_r=0$; terminal control is derived on page 3.',True)

page('2    Wheel PI gains by discrete pole placement')
p(r'Existing model inputs: $J=0.001125\,\mathrm{kg\,m^2}$ per drive wheel; sample interval $h=0.01\,\mathrm{s}$. Selected inner-loop poles: $p_1=0.8$, $p_2=0.5$. Constant reference/load, unsaturated scalar wheel model.')
eq(r'J\dot\Omega=\tau-d,\quad e_k=\Omega^*-\Omega_k,\quad I_k=I_{k-1}+K_ihe_k,\quad\tau_k=K_pe_k+I_k','PI: Åström & Murray, Feedback Systems (2020), Eq. (11.1), p. 11-2.')
eq(r'\widetilde I_k=I_k-d,\quad e_{k+1}=e_k-\frac hJ(K_pe_k+\widetilde I_k)')
eq(r'a=\frac{hK_p}{J},\quad b=\frac{h^2K_i}{J},\quad e_{k+1}=(1-a-b)e_k-\frac hJ\widetilde I_{k-1}')
eq(r'\widetilde I_k=K_ihe_k+\widetilde I_{k-1},\quad\mathrm{tr}(A)=2-a-b,\quad\det A=1-a')
eq(r'\det(zI-A)=z^2-(2-a-b)z+(1-a)','V2 derivation for the implemented PI update order.')
eq(r'(z-p_1)(z-p_2)=(z-0.8)(z-0.5)=z^2-1.3z+0.4')
eq(r'1-a=0.4\Rightarrow a=0.6,\quad 2-a-b=1.3\Rightarrow b=0.1')
eq(r'K_p=\frac{aJ}{h}=\frac{0.6(0.001125)}{0.01}=0.0675\ \mathrm{N\,m\,s/rad}')
eq(r'K_i=\frac{bJ}{h^2}=\frac{0.1(0.001125)}{0.01^2}=1.125\ \mathrm{N\,m/rad}')
eq(r'|p_1|=0.8<1,\quad|p_2|=0.5<1\quad\Rightarrow\quad e_k\to0,\ I_k\to d','Åström & Murray, Feedback Systems (2020), Exercise 6.8(b), p. 6-37.')
p('Command conversion and implemented anti-windup')
eq(r'\Omega_L^*=\frac{v_c-b_d\omega_c/2}{r_d},\quad\Omega_R^*=\frac{v_c+b_d\omega_c/2}{r_d},\quad b_d=0.38\ \mathrm m,\ r_d=0.075\ \mathrm m')
eq(r'I_{\rm cand}=I_{k-1}+K_ihe_k,\quad\tau_{\rm raw}=K_pe_k+I_{\rm cand},\quad\tau_s=\mathrm{clip}(\tau_{\rm raw},-0.5,0.5)','Conditional integration adapted from Feedback Systems, Exercise 11.11, p. 11-26.')
p(r'Use $I_k=I_{\mathrm{cand}}$ if $|\tau_{\mathrm{raw}}|\leq0.5$ or $e_k(\tau_{\mathrm{raw}}-\tau_s)\leq0$; otherwise $I_k=I_{k-1}$. Apply $\tau_k=\mathrm{clip}(K_pe_k+I_k,-0.5,0.5)$. The pole calculation above applies while unsaturated.',True)
p(r'First step from rest for the straight reference: $v_c=0.15\,\mathrm{m/s}$, $\omega_c=0$, $I_{-1}=0$.')
eq(r'\Omega_L^*=\Omega_R^*=\frac{0.15}{0.075}=2\ \mathrm{rad/s},\quad e_0=2\ \mathrm{rad/s}')
eq(r'I_0=1.125(0.01)(2)=0.0225\ \mathrm{N\,m}')
eq(r'\tau_0=0.0675(2)+0.0225=0.1575\ \mathrm{N\,m}<0.5\ \mathrm{N\,m}')

page('3    Terminal controller and its gain calculation')
p(r'Selected terminal targets: aligned-position time constant $\tau_\rho=0.5\,\mathrm{s}$; bearing and final-heading time constants $\tau_\alpha=\tau_g=\frac{1}{3}\,\mathrm{s}$. Fixed goal; nominal unsaturated axle model; $\rho>0$.')
eq(r'\delta=p_g-p_A,\quad\rho=\|\delta\|,\quad\beta=\mathrm{atan2}(\delta_y,\delta_x)')
eq(r's\in\{-1,+1\},\quad\theta_e=\theta+\frac{1-s}{2}\pi,\quad\alpha=\mathrm{wrap}(\beta-\theta_e),\quad|\alpha_0|\leq\pi/2')
eq(r'u=sv,\quad\dot\rho=-u\cos\alpha,\quad\dot\alpha=\frac u\rho\sin\alpha-\omega','Siegwart & Nourbakhsh, Autonomous Mobile Robots, Ch. 3 slides, §3.6.2, PDF p. 36.')
eq(r'v_c=s k_\rho\rho\cos\alpha,\qquad\omega_c=k_\alpha\alpha+k_\rho\sin\alpha\cos\alpha','V2 derivation: cancellation using the polar kinematics above.')
eq(r'\dot\rho=-k_\rho\rho\cos^2\alpha,\qquad\dot\alpha=-k_\alpha\alpha')
eq(r'\alpha=0\Rightarrow\dot\rho=-\rho/\tau_\rho\Rightarrow k_\rho=1/\tau_\rho=2\ \mathrm{s^{-1}}','Åström & Murray, Feedback Systems (2020), Exercise 6.7, p. 6-36.')
eq(r'\dot\alpha=-\alpha/\tau_\alpha\Rightarrow k_\alpha=1/\tau_\alpha=3\ \mathrm{s^{-1}}')
eq(r'v_c=s\,2\rho\cos\alpha,\quad\omega_c=3\alpha+2\sin\alpha\cos\alpha,\quad\alpha(t)=\alpha_0e^{-3t}')
eq(r'\rho(t)=\rho_0\exp\!\left[-2\int_0^t\cos^2\alpha(\xi)\,d\xi\right]')
eq(r't_1=\frac13\ln\max\!\left(1,\frac{4|\alpha_0|}{\pi}\right),\quad t\geq t_1\Rightarrow\cos^2\alpha\geq\frac12')
eq(r'0\leq\rho(t)\leq\rho(t_1)e^{-(t-t_1)}\to0')
p(r'Final heading: $v_c=0$; local target $\dot e_g=-e_g/\tau_g$, $|e_g|<\pi$.')
eq(r'e_g=\mathrm{wrap}(\theta_g-\theta),\quad\omega_c=k_g\sin e_g,\quad\dot e_g=-k_g\sin e_g\simeq-k_ge_g')
eq(r'k_g=1/\tau_g=3\ \mathrm{s^{-1}},\quad V_g=1-\cos e_g,\quad\dot V_g=-3\sin^2e_g\leq0','Lyapunov method: Feedback Systems (2020), Theorem 5.2; V2 substitution.')
p('Calculated controller gains used in the implementation:',True)
eq(r'(k_x,k_y,k_\theta)=(2,44.444444,2),\quad(K_p,K_i)=(0.0675,1.125),\quad(k_\rho,k_\alpha,k_g)=(2,3,3)')
p('These are nominal design calculations. Saturation, wheel slip, time-varying loads and switching require separate closed-loop verification; the calculation is not a global stability proof of the complete M2 robot.',True)

expressions=[s.replace(r'\frac12',r'\frac{1}{2}').replace(r'\frac13',r'\frac{1}{3}').replace(r'\frac1{',r'\frac{1}{').replace(r'\frac1{',r'\frac{1}{').replace(r'\frac hJ',r'\frac{h}{J}').replace(r'\frac u\rho',r'\frac{u}{\rho}').replace(r'\mathrm m',r'\mathrm{m}') for s in expressions]
(TEMP/'equations.json').write_text(json.dumps(expressions,ensure_ascii=False))
subprocess.run([str(ROOT.parent/'caster-physics-python/.venv/bin/python'),str(ROOT/'docs/render_math_assets.py'),str(TEMP/'equations.json'),str(TEMP)],check=True)
def footer(canvas,document):
    canvas.setFont('Body',8);canvas.setFillColor(colors.HexColor('#566674'))
    canvas.drawRightString(543,26,f'{document.page} / 3')
destination=ROOT/'output/pdf/Arche_V2_Controller_Design_EN.pdf'
doc=SimpleDocTemplate(str(destination),pagesize=(595.28,841.89),leftMargin=52,rightMargin=52,topMargin=40,bottomMargin=44,title='V2 controller design calculations',author='')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
assert len(PdfReader(destination).pages)==3
print(destination)
