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
TEMP = ROOT / "tmp/pdfs/mechanical-derivation-en"
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



page('1    Geometry, kinematics and support loads')
eq(r'\psi=\theta+\phi,\quad\Omega=\dot\theta,\quad\varrho=\dot\phi,\quad e(\eta)=(\cos\eta,\sin\eta)^T')
eq(r'n(\eta)=(-\sin\eta,\cos\eta)^T,\quad E=[\ (0,1)^T\ \ (-1,0)^T\ ],\quad E^2=-I_2')
eq(r'c=\overrightarrow{AB},\quad h=\overrightarrow{AH},\quad p_A=p_B-R(\theta)c,\quad r_H=R(\theta)(h-c)')
eq(r'r_C=r_H-\ell e(\psi),\quad r_L=R(\theta)((0,b/2)^T-c),\quad r_R=R(\theta)((0,-b/2)^T-c)')
eq(r'p_i=p_B+r_i,\quad\dot R=\Omega ER,\quad\dot e(\psi)=(\Omega+\varrho)n(\psi)')
eq(r'v_L=v_B+\Omega Er_L,\qquad v_R=v_B+\Omega Er_R')
eq(r'v_C=v_B+\Omega Er_H-\ell(\Omega+\varrho)n(\psi)=v_B+\Omega Er_C-\ell\varrho n(\psi)')
eq(r'a_C=a_B+Er_H\dot\Omega-\Omega^2r_H-\ell n(\psi)(\dot\Omega+\dot\varrho)+\ell(\Omega+\varrho)^2e(\psi)')
eq(r'a_C=a_B+Er_C\dot\Omega-\ell n(\psi)\dot\varrho+b_C,\quad b_C=-\Omega^2r_H+\ell(\Omega+\varrho)^2e(\psi)')
p('Quasi-static vertical balance, as in the supplied derivation')
eq(r'x_C=h_x-\ell\cos\phi,\qquad y_C=h_y-\ell\sin\phi')
eq(r'N_L+N_R+N_C=(m_b+m_c)g')
eq(r'x_CN_C=g(m_bc_x+m_cx_C)')
eq(r'\frac b2(N_L-N_R)+y_CN_C=g(m_bc_y+m_cy_C)')
eq(r'N_C=\frac{g(m_bc_x+m_cx_C)}{x_C},\qquad S_N=(m_b+m_c)g-N_C')
eq(r'\Delta N=\frac2b\left[g(m_bc_y+m_cy_C)-y_CN_C\right]')
eq(r'N_L=\frac{S_N+\Delta N}{2},\quad N_R=\frac{S_N-\Delta N}{2},\quad x_C\ne0,\ N_L,N_R,N_C>0')

page('2    Coupled equations of motion')
p('H: pivot force on caster; τ_s: bearing resistance; τ₀: ground bore resistance')
eq(r'm_ba_B=F_L+F_R-H,\qquad m_ca_C=H+F_C')
eq(r'I_B\dot\Omega=r_L\times F_L+r_R\times F_R-r_H\times H+\tau_s')
eq(r'J_s(\dot\Omega+\dot\varrho)=\ell e(\psi)\times H-\tau_s-\tau_0')
eq(r'H=m_ca_C-F_C\quad\Rightarrow\quad m_ba_B+m_ca_C=F_L+F_R+F_C')
eq(r'J_s(\dot\Omega+\dot\varrho)-m_c\ell n(\psi)^Ta_C=-\ell n(\psi)^TF_C-\tau_s-\tau_0')
eq(r'J_d\dot\omega_L=\tau_L-r_df_{L\parallel}-\tau_{rL},\quad J_d\dot\omega_R=\tau_R-r_df_{R\parallel}-\tau_{rR}')
eq(r'J_c\dot\omega_C=-r_cf_{C\parallel}-\tau_{rC}')
eq(r'\nu=(v_{Bx},v_{By},\Omega,\varrho,\omega_L,\omega_R,\omega_C)^T')
eq(r'J_C=[\ I_2\ \ Er_C\ \ -\ell n(\psi)\ \ 0\ \ 0\ \ 0\ ],\qquad v_C=J_C\nu')
eq(r'J_i=[\ I_2\ \ Er_i\ \ 0\ \ 0\ \ 0\ \ 0\ ],\quad i=L,R,\qquad a_C=J_C\dot\nu+b_C')
eq(r'a_s=(0,0,1,1,0,0,0)^T')
eq(r'M=\mathrm{diag}(m_b,m_b,I_B,0,J_d,J_d,J_c)+m_cJ_C^TJ_C+J_sa_sa_s^T')
eq(r'b_q=m_cJ_C^Tb_C,\qquad M\dot\nu+b_q=Q')
p('u_j: j-th unit vector in the seven-dimensional velocity space')
eq(r'A_L=J_L-r_de(\theta)u_5^T,\quad A_R=J_R-r_de(\theta)u_6^T,\quad A_C=J_C-r_ce(\psi)u_7^T')
eq(r'Q=A_L^TF_L+A_R^TF_R+A_C^TF_C+u_5(\tau_L-\tau_{rL})+u_6(\tau_R-\tau_{rR})')
eq(r'\phantom{Q=}-u_7\tau_{rC}-u_4\tau_s-(u_3+u_4)\tau_0')
eq(r'\dot\nu=\mathrm{solve}(M,Q-b_q)')

page('3    Contact passivity and energy balance')
p('Contact law from the supplied derivation - symbolic coefficients')
eq(r'r_{wL}=r_{wR}=r_d,\quad r_{wC}=r_c,\quad e_{L,R}=e(\theta),\ e_C=e(\psi),\ n_i=Ee_i')
eq(r's_i=(e_i^Tv_i-r_{wi}\omega_i,\ n_i^Tv_i)^T,\quad F_i=[\ e_i\ \ n_i\ ]f_i')
eq(r'L_i=\mu_iN_i,\quad z_i=(k_{i\parallel}s_{i\parallel},k_{i\perp}s_{i\perp})^T,\quad f_i=-\frac{L_iz_i}{\sqrt{L_i^2+\|z_i\|^2}}\quad(i=L,R)')
eq(r'\|f_i\|\leq L_i,\qquad f_i^Ts_i=-\frac{L_i(k_{i\parallel}s_{i\parallel}^2+k_{i\perp}s_{i\perp}^2)}{\sqrt{L_i^2+\|z_i\|^2}}\leq0')
eq(r'L=\mu_CN_C,\quad z_C=(k_{C\parallel}s_{C\parallel},k_{C\perp}s_{C\perp},L(\Omega+\varrho)/\epsilon_0)^T')
eq(r'\lambda=\frac L{\sqrt{L^2+\|z_C\|^2}},\quad f_C=-\lambda(z_{C1},z_{C2})^T,\quad\tau_0=\lambda z_{C3}r_0')
eq(r'\sqrt{\|f_C\|^2+(\tau_0/r_0)^2}\leq L\quad(r_0>0)')
eq(r'\tau_{ri}=C_{rri}N_ir_{wi}\tanh(\omega_i/\epsilon_r)+b_{ri}\omega_i,\quad\tau_s=b_s\varrho+\tau_{s0}\tanh(\varrho/\epsilon_s)')
eq(r'-f_C^Ts_C+\tau_0(\Omega+\varrho)=\lambda\!\left[k_{C\parallel}s_{C\parallel}^2+k_{C\perp}s_{C\perp}^2+\frac{Lr_0}{\epsilon_0}(\Omega+\varrho)^2\right]\geq0')
eq(r'r_0=0\Rightarrow z_{C3}=\tau_0=0,\quad L=0\Rightarrow f_C=0,\ \tau_0=0,\quad L_i=0\Rightarrow f_i=0')
p('Fixed-height energy balance - positive masses, inertias and regularization scales; nonnegative loss coefficients')
eq(r'T=\frac12m_b\|v_B\|^2+\frac12I_B\Omega^2+\frac12m_c\|v_C\|^2+\frac12J_s(\Omega+\varrho)^2')
eq(r'\phantom{T=}+\frac12J_d(\omega_L^2+\omega_R^2)+\frac12J_c\omega_C^2=\frac12\nu^TM\nu')
eq(r'\nu^Tb_q=\frac12\nu^T\dot M\nu\quad\Rightarrow\quad\dot T=\nu^T(M\dot\nu+b_q)=\nu^TQ')
eq(r'P_{\rm in}=\tau_L\omega_L+\tau_R\omega_R,\quad P_{\rm diss}=-\sum_i f_i^Ts_i+\sum_i\tau_{ri}\omega_i+\tau_s\varrho+\tau_0(\Omega+\varrho)\geq0')
eq(r'\dot T=P_{\rm in}-P_{\rm diss},\quad\dot W=P_{\rm in},\quad\dot D=P_{\rm diss},\quad W(0)=D(0)=0')
eq(r'T(t)-T(0)+D(t)-W(t)=0')
eq(r'm_b,m_c,I_B,J_s,J_d,J_c>0:\quad\nu^TM\nu=2T=0\Rightarrow\nu=0\Rightarrow M\succ0')

expressions=[s.replace(r'\frac12',r'\frac{1}{2}').replace(r'\frac b2',r'\frac{b}{2}').replace(r'\frac2b',r'\frac{2}{b}').replace(r'\frac L{',r'\frac{L}{').replace(r'\ne0',r'\neq0') for s in expressions]
(TEMP/'equations.json').write_text(json.dumps(expressions,ensure_ascii=False))
subprocess.run([str(ROOT.parent/'caster-physics-python/.venv/bin/python'),str(ROOT/'docs/render_math_assets.py'),str(TEMP/'equations.json'),str(TEMP)],check=True)
def footer(canvas,document):
    canvas.setFont('Body',8);canvas.setFillColor(colors.HexColor('#566674'))
    canvas.drawRightString(543,26,f'{document.page} / 3')
destination=ROOT/'output/pdf/Arche_Mechanical_Derivation_EN.pdf'
doc=SimpleDocTemplate(str(destination),pagesize=(595.28,841.89),leftMargin=52,rightMargin=52,topMargin=40,bottomMargin=44,title='Mechanical derivation',author='')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
assert len(PdfReader(destination).pages)==3
print(destination)
