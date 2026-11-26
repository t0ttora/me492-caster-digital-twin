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
TEMP = ROOT / "tmp/pdfs/short-calculations-en"
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


page('1 / 3    Wheel inertia and PI calculation')
p('Assumption from your V1 sheet: each drive wheel is a uniform solid cylinder, m=0.4 kg, r=0.075 m. About its own spin axis, integrate concentric rings (surface density σ=m/(πr²)):')
eq(r'J=\int_0^r \rho^2(2\pi\sigma\rho\,d\rho)=\frac{\pi\sigma r^4}{2}=\frac12mr^2')
eq(r'J_d=\frac12(0.4)(0.075)^2=0.001125\ \mathrm{kg\,m^2},\quad J_c=\frac12(0.4)(0.040)^2=0.000320\ \mathrm{kg\,m^2}')
p('Python inherited J_d=0.0045 and J_c=0.00045 from the provisional MATLAB model. The drive value is four times your cylinder result. For any 0.4 kg wheel contained within 75 mm of the spin axis, J≤mr²=0.00225; 0.0045 therefore cannot represent that wheel alone. No documented reflected motor/gear inertia was found. The exact origin of the old number is unproven.')
p('Decision: use J_d=0.001125 per drive wheel and J_c=0.000320 for caster roll in the current V2 parameters. Preserve V1/MATLAB as historical references. Body/caster yaw inertias are separate and remain provisional. Radius sweeps scale caster roll inertia as r² at fixed mass.')
eq(r'\tau=J_d\dot\Omega,\quad \dot\Omega=0.01/0.001125=8.888888889\ \mathrm{rad/s^2}')
eq(r'\Omega(0.2)=0+(8.888888889)(0.2)=1.777777778\ \mathrm{rad/s}')
p('This matches your isolated-wheel example: fixed axis, zero initial speed, no ground contact or losses. The old J_d gives only 0.444444444 rad/s. This is a wheel test, not the full robot.')
p('Wheel PI: e=Ω*−Ω, K_p=0.15 N m s/rad, K_i=0.8 N m/rad, Δt=0.01 s; I stores integral torque.')
eq(r'I_{\rm cand}=I_{k-1}+0.8\Delta t\,e_k,\quad\tau_{\rm raw}=0.15e_k+I_{\rm cand},\quad\tau_s=\mathrm{clip}(\tau_{\rm raw},-0.5,0.5)')
p('Accept I_cand if |τ_raw|≤0.5 or e_k(τ_raw−τ_s)≤0; otherwise hold I. Apply τ=clip(0.15e+I,±0.5). For constant Ω*, constant load d and no saturation, set Ĩ=I−d in JΩ̇=τ−d:')
eq(r'J\dot e=-K_pe-\widetilde I,\quad\dot{\widetilde I}=K_ie,\quad V_i=\frac12Je^2+\frac{\widetilde I^2}{2K_i},\quad\dot V_i=-K_pe^2\leq0')
p('On the invariant set e=0, ė=0 requires Ĩ=0: speed error tends to zero and I tends to d. For the actual update order, the unsaturated scalar sampled system has a=ΔtK_p/J=1.333333, b_i=Δt²K_i/J=0.071111. Jury: a>0, b_i>0, 4−2a−b_i=1.262222>0; poles are −0.351841 and 0.947397, inside the unit circle.',True)
p('Numerical PI check, J=0.001125, d=(0.08,0.10), Ω*=(2,2), Ω₀=I₋₁=0: step 0 gives I=(.016,.016), τ=(.316,.316), Ω₁=(2.097778,1.920000). Step 1 gives I=(.015218,.016640), τ=(.000551,.028640), Ω₂=(1.391565,1.285689). At 400 steps, Ω→(2,2), I→d.',True)
p('Sources: supplied V1 PDF p.5; frozen V1 dynamics_si.json; V2 parameters/dynamics_si.json. Cylinder derivation also follows OpenStax, University Physics Vol.1, §10.5. These are assumed geometric inertias, not hardware measurements.',True)

page('2 / 3    Tracking control calculation')
p('Your V1 notation is retained: c=AB, h=AH, E=[0 −1;1 0], p_A=p_B−R(θ)c. The mechanics remain M(q)ν̇+b(q,ν)=Q(q,ν,τ); only the two spin inertias above are aligned. θ is body yaw, ω=θ̇; Ω_L,Ω_R are wheel rates.')
p('For target axle pose (p_r,θ_r) and reference velocities v_r,ω_r, define body-frame errors:')
eq(r'(e_x,e_y)^T=R(\theta)^T(p_r-p_A),\qquad e_\theta=\theta_r-\theta')
p('Using ideal axle kinematics ṗ_A=v(cosθ,sinθ)ᵀ and Ṙᵀ=−ωERᵀ, differentiate:')
eq(r'\dot e_x=\omega e_y-v+v_r\cos e_\theta,\quad\dot e_y=-\omega e_x+v_r\sin e_\theta,\quad\dot e_\theta=\omega_r-\omega')
eq(r'v_c=v_r\cos e_\theta+2e_x,\qquad\omega_c=\omega_r+6v_re_y+3\sin e_\theta')
p('Assume no saturation and perfect velocity realization, v=v_c and ω=ω_c. Choose V and substitute the error derivatives:')
eq(r'V=\frac12(e_x^2+e_y^2)+\frac{1-\cos e_\theta}{6}')
eq(r'\dot V=e_x(\omega e_y-v+v_r\cos e_\theta)+e_y(-\omega e_x+v_r\sin e_\theta)+\frac{\sin e_\theta}{6}(\omega_r-\omega)')
eq(r'\dot V=-2e_x^2+v_re_y\sin e_\theta-\frac{\sin e_\theta}{6}(6v_re_y+3\sin e_\theta)')
eq(r'\dot V=-2e_x^2-\frac12\sin^2e_\theta\leq0')
p('Hand check: e_x=.01 m, e_y=−.005 m, e_θ=.1 rad, v_r=.15 m/s, ω_r=π/6 rad/s.')
eq(r'v_c=.15\cos(.1)+2(.01)=0.169250625\ \mathrm{m/s}')
eq(r'\omega_c=\pi/6+6(.15)(-.005)+3\sin(.1)=0.818599026\ \mathrm{rad/s}')
eq(r'\dot V=-2(.01)^2-\frac12\sin^2(.1)=-0.005183356\ \mathrm{m^2/s}')
p('Convert to wheel references with b=.38 m and r_d=.075 m:')
eq(r'\Omega_L^*=\frac{v_c-b\omega_c/2}{r_d},\qquad\Omega_R^*=\frac{v_c+b\omega_c/2}{r_d}')
p('The gains have units 2 s⁻¹, 6 m⁻² and 3 s⁻¹. Non-increasing V does not by itself prove e_y→0. In particular, v_r=0 removes the lateral correction; terminal position control is therefore required.',True)

page('3 / 3    Terminal position and heading')
p('After the profile ends, the goal (p_g,θ_g) is fixed. Set ρ=‖p_g−p_A‖ and β=atan2(y_g−y_A,x_g−x_A). Choose forward/reverse direction s∈{+1,−1} on entry so |α₀|≤π/2; hold s during this approach:')
eq(r'\theta_e=\theta+\frac{1-s}{2}\pi,\qquad\alpha=\mathrm{wrap}(\beta-\theta_e)')
eq(r'v_c=s\,2\rho\cos\alpha,\qquad\omega_c=3\alpha+2\sin\alpha\cos\alpha')
p('For an ideal unsaturated axle with effective speed u=s v, and ρ>0:')
eq(r'\dot\rho=-u\cos\alpha,\qquad\dot\alpha=\frac{u}{\rho}\sin\alpha-\omega')
eq(r'\dot\rho=-2\rho\cos^2\alpha,\qquad\dot\alpha=-3\alpha\quad\Rightarrow\quad\alpha(t)=\alpha_0e^{-3t}')
eq(r'\rho(t)=\rho_0\exp\!\left[-2\int_0^t\cos^2\alpha(\xi)\,d\xi\right]\longrightarrow0')
p('Reason: after finite t₁, |α|≤π/4, hence cos²α≥1/2. Therefore ρ̇≤−ρ and ρ(t)≤ρ(t₁)exp[−(t−t₁)] for t≥t₁. This proves nominal position convergence, not the final heading.')
p('Hand check: p_g−p_A=(.030,.001) m, θ=0, s=+1:')
eq(r'\rho=0.030016662\ \mathrm{m},\quad\alpha=\arctan(1/30)=0.033320996\ \mathrm{rad}')
eq(r'v_c=0.060000000\ \mathrm{m/s},\qquad\omega_c=0.166555662\ \mathrm{rad/s}')
p('At ρ≤6 mm, align heading: e_g=wrap(θ_g−θ), v_c=0, ω_c=3 sin(e_g).')
eq(r'V_g=1-\cos e_g,\qquad\dot V_g=-3\sin^2e_g\leq0\qquad(|e_g|<\pi)')
p('Ideal in-place rotation preserves position and locally stabilizes heading, excluding the antipodal equilibrium. In M2, slip can move the axle: return to position approach if ρ>12 mm. All modes enforce |v_c|≤.25 m/s, |ω_c|≤1 rad/s and |τ|≤.5 N m.')
p('Completion requires position≤20 mm, |yaw error|≤.05 rad, planar speed≤.02 m/s and |yaw rate|≤.05 rad/s, continuously for .25 s. Profile time and terminal recovery time are reported separately.',True)
p('Scope: these calculations prove properties of the stated ideal submodels. They do not prove global stability of the complete slipping, saturated, sampled hybrid M2 system. Other masses, yaw inertias and contact coefficients remain provisional. The current V2 uses the corrected wheel inertias; frozen V1 and previous PDFs/results retain their historical values.',True)

expressions=[s.replace(r'\frac12',r'\frac{1}{2}') for s in expressions]
(TEMP/'equations.json').write_text(json.dumps(expressions,ensure_ascii=False))
subprocess.run([str(ROOT.parent/'caster-physics-python/.venv/bin/python'),str(ROOT/'docs/render_math_assets.py'),str(TEMP/'equations.json'),str(TEMP)],check=True)
def footer(canvas,document):
    canvas.setFont('Body',7.5);canvas.setFillColor(colors.HexColor('#566674'))
    canvas.drawString(52,26,'Arche V2.2 | Inertia-aligned calculations | 13 November 2026')
    canvas.drawRightString(543,26,f'{document.page} / 3')
destination=ROOT/'output/pdf/Arche_V2_Calculations_EN.pdf'
doc=SimpleDocTemplate(str(destination),pagesize=(595.28,841.89),leftMargin=52,rightMargin=52,topMargin=44,bottomMargin=46,title='Arche V2 inertia-aligned calculations',author='Arche ME492')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
reader=PdfReader(destination)
assert len(reader.pages)==3,len(reader.pages)
assert all(len(p.extract_text())>300 for p in reader.pages)
print(destination)
