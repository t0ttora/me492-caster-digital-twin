"""Build the Turkish hand-verification report using the bundled PDF runtime."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.utils import ImageReader
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Flowable, Image
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output/pdf"
TEMP = ROOT / "tmp/pdfs/mathematical-report"
OUTPUT.mkdir(parents=True, exist_ok=True)
TEMP.mkdir(parents=True, exist_ok=True)
FONT = Path("/Users/tora/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/poppler/fonts")
pdfmetrics.registerFont(TTFont("Body", str(FONT / "DejaVuSans.ttf")))
bold = FONT / "DejaVuSans-Bold.ttf"
pdfmetrics.registerFont(TTFont("BodyBold", str(bold if bold.exists() else FONT / "DejaVuSans.ttf")))
pdfmetrics.registerFontFamily("Body", normal="Body", bold="BodyBold", italic="Body", boldItalic="BodyBold")
styles = {
    "body": ParagraphStyle("body", fontName="Body", fontSize=9.4, leading=14.1, spaceAfter=8, textColor=colors.HexColor("#253541")),
    "h1": ParagraphStyle("h1", fontName="BodyBold", fontSize=19, leading=24, spaceAfter=15, textColor=colors.HexColor("#132b3a")),
    "h2": ParagraphStyle("h2", fontName="BodyBold", fontSize=11, leading=15, spaceBefore=7, spaceAfter=7, textColor=colors.HexColor("#132b3a")),
    "small": ParagraphStyle("small", fontName="Body", fontSize=8, leading=11, spaceAfter=6),
    "cell": ParagraphStyle("cell", fontName="Body", fontSize=8, leading=11),
}
story, equations = [], []
page_titles = []


class Equation(Flowable):
    def __init__(self, expression, number):
        super().__init__()
        self.index = len(equations)
        equations.append(expression)
        self.number = str(number)

    def wrap(self, width, height):
        self.path = TEMP / f"eq-{self.index:03d}.png"
        with PILImage.open(self.path) as image:
            iw, ih = image.size
        scale = min(72/240, (width-56)/iw)
        self.image_width, self.image_height = iw*scale, ih*scale
        self.width, self.height = width, self.image_height+15
        return self.width, self.height

    def draw(self):
        self.canv.drawImage(str(self.path), 3, 7, width=self.image_width, height=self.image_height, mask="auto")
        self.canv.setFont("Body", 8)
        self.canv.setFillColor(colors.HexColor("#60717c"))
        self.canv.drawRightString(self.width, self.height/2, f"({self.number})")


def page(title):
    if story:
        story.append(PageBreak())
    page_titles.append(title)
    story.append(Paragraph(title, styles["h1"]))


def p(text, kind="body"):
    story.append(Paragraph(text, styles[kind]))


def eq(expression, number):
    story.append(Equation(expression, number))


def table(rows, widths=None, font_size=8):
    cells = [[Paragraph(str(value), styles["cell"]) for value in row] for row in rows]
    item = Table(cells, colWidths=widths, hAlign="LEFT", repeatRows=1)
    item.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#eaf0f3")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6), ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("LINEBELOW", (0, 0), (-1, 0), .6, colors.HexColor("#96aab5")),
        ("LINEBELOW", (0, 1), (-1, -1), .2, colors.HexColor("#dce3e7")),
    ]))
    story.append(item)
    story.append(Spacer(1, 9))


def numeric_matrix(label, matrix):
    p(label, "h2")
    values = [[f"{v:.9f}" for v in row] for row in matrix]
    item = Table(values, colWidths=[67]*len(matrix[0]), hAlign="LEFT")
    item.setStyle(TableStyle([("FONTNAME", (0,0),(-1,-1),"Body"), ("FONTSIZE",(0,0),(-1,-1),7.1),
                             ("ALIGN",(0,0),(-1,-1),"RIGHT"), ("TOPPADDING",(0,0),(-1,-1),4),
                             ("BOTTOMPADDING",(0,0),(-1,-1),4),
                             ("BACKGROUND",(0,0),(-1,-1),colors.HexColor("#f3f6f8"))]))
    story.append(item)
    story.append(Spacer(1, 10))


sample = json.loads((ROOT / "evidence/hand-verification-example.json").read_text())
before = json.loads((ROOT / "outputs/validation-2026-11-13/study.json").read_text())
after = json.loads((ROOT / "outputs/tracking-fix-v2.1-2026-11-13/study.json").read_text())
nominal = [run for run in after["runs"] if run["design"] == "nominal"]
completed = sum(run["metrics"]["completion_time_s"] is not None for run in nominal)
same_window = sum(run["metrics"]["completion_within_original_1s_window_s"] is not None for run in nominal)
all_completed = sum(run["metrics"]["completion_time_s"] is not None for run in after["runs"])

page("Arche V2 matematiksel türetim ve doğrulama")
p("13 Kasım 2026 · V2.1 · Kâğıt üzerinde yeniden çözüm için teknik not", "h2")
p("Bu not, V1'deki düzlemsel caster modelinin denklemlerini ve V2.1 takip kontrolünü adım adım yeniden kurar. Amaç; kodun çıktısına güvenmek yerine, aynı geometri ve varsayımlardan başlayıp kuvvetleri, ivmeleri ve kontrol özdeşliklerini bağımsız olarak hesaplayabilmendir.")
p("<b>Sonuç:</b> önceki 16 nominal koşunun 3'ü bitiyordu. Düzeltilen kontrolde aynı 1 saniyelik son gözlem penceresinde "
  f"{same_window}/16; açıkça tanımlanmış 8 saniyelik toparlanma penceresinde {completed}/16 koşu bitiş koşulunu sağladı. "
  f"Duyarlılık koşuları dahil {all_completed}/40 tamamlandı. Konum/açı eşikleri gevşetilmedi; hız kontrolü düzlemdeki toplam hızı da denetler.")
table([["Matematiksel olarak gösterilen", "Sınır"],
       ["M simetrik pozitif tanımlıdır; mekanik denklem ve enerji özdeşliği türetilir.", "Seçilen düzlemsel, toplulaştırılmış mekanik model için."],
       ["Takip Lyapunov türevi, terminal yaklaşma ve PI yük reddi ispatlanır.", "İlgili ideal sürekli-zaman ve doygunluksuz varsayımlar altında."],
       ["Doygun, örneklemeli tam M2 sisteminin 40 koşusu sayısal olarak sınanır.", "Tam M2 için küresel kapalı çevrim kararlılık ispatı veya gerçek robot doğrulaması değildir."]], [239,239])
p("Model parametreleri sentetiktir. Doğru bir denklem türetimi, bu sayıların gerçek platforma ait olduğunu göstermez. V1 kaynak, parametre ve referans dosyaları değiştirilmedi.")
p("Okuma sırası: 2-8 mekanik model; 9 eski hatanın nedeni; 10-14 kontrol ve ispatlar; 15-18 sayısal elle çözüm; 19-20 deney ve yeniden üretim.")

page("2 Koordinatlar ve varsayımlar")
p("Zemin düz; yükseklik, roll ve pitch sabit. Düşey destek quasi-static; ivmeden yük transferi yok. Lastik elastisitesi, darbe, devrilme, motor elektrik devresi ve fiziksel shimmy bu modelin dışında. Gövde kütlesi caster dışındaki hareketli kütleyi; caster kütlesi tekerlek merkezindeki toplulaştırmayı temsil eder.")
table([["İşaret", "Tanım ve SI birimi"], ["A, B, C, P", "Tahrik aksı merkezi, gövde COM, caster merkezi, caster pivotu"],
       ["θ, φ, ψ=θ+φ", "Gövde yaw, gövdeye göre caster açısı, mutlak caster açısı [rad]"],
       ["a, p, l, b", "A'dan gövde COM ve pivot ofseti [m], caster trail [m], tahrik izi [m]"],
       ["r_d, r_c", "Tahrik ve caster yuvarlanma yarıçapı [m]"],
       ["ν", "[v_Bx, v_By, θ̇, φ̇, Ω_L, Ω_R, Ω_C]ᵀ"],
       ["m_b,m_c; I_b,I_s,I_d,I_w", "Kütleler [kg]; gövde yaw, caster swivel, tahrik roll, caster roll ataletleri [kg m²]"]], [130,348])
eq(r"q=(x_B,y_B,\theta,\phi,\gamma_L,\gamma_R,\gamma_C)^T,\qquad \dot q=\nu", "2.1")
p("γ tekerlek roll açıları çevrimseldir: T ve kuvvetler bu açıların kendisine bağlı değildir; hızlarını entegre etmek yeterlidir. Kodun 13 durumu dört geometrik durum, yedi hız ve W,D enerji integralleridir.")
eq(r"R(\theta)=\left[\,e_\theta\ \ n_\theta\,\right],\quad e_\eta=(\cos\eta,\sin\eta)^T,\quad n_\eta=(-\sin\eta,\cos\eta)^T", "2.2")
eq(r"S(x,y)^T=(-y,x)^T,\qquad p_A=p_B-R(\theta)a", "2.3")
p("Poz referansı A içindir; entegre edilen dünya koordinatı B içindir. Bu ofseti atlamak, kontrol hatasını ve sayısal örneği değiştirir.")

page("3 Geometri ve hız Jacobianları")
eq(r"d=R(\theta)(p-a),\qquad r=d-l e_\psi,\qquad p_C=p_B+r", "3.1")
p("Sabit ofsetler gövde düzlemindedir. Caster'ın açısal hızı ψ̇=θ̇+φ̇ olduğundan, caster konumunu zamanla türet:")
eq(r"v_C=v_B+S d\,\dot\theta-l n_\psi(\dot\theta+\dot\phi)=v_B+S r\,\dot\theta-l n_\psi\dot\phi", "3.2")
eq(r"J_C=[\ I_2\ \ Sr\ \ -l n_\psi\ \ 0\ \ 0\ \ 0\ ],\qquad v_C=J_C\nu", "3.3")
eq(r"r_L=R(\theta)((0,b/2)^T-a),\qquad r_R=R(\theta)((0,-b/2)^T-a)", "3.4")
eq(r"J_L=[\ I_2\ \ Sr_L\ \ 0\ \ 0\ \ 0\ \ 0\ ],\quad J_R=[\ I_2\ \ Sr_R\ \ 0\ \ 0\ \ 0\ \ 0\ ]", "3.5")
p("J matrisleri 2×7'dir. İlk iki sütun dünya COM hızlarını; üçüncü sütun gövde yaw etkisini; dördüncü sütun göreli caster dönmesini haritalar. Yuvarlanma hızları tekerlek merkezini doğrudan ötelemez, temas kaymasına girer.")
eq(r"s_L=(e_\theta^TJ_L\nu-r_d\Omega_L,\ n_\theta^TJ_L\nu)^T", "3.6")
eq(r"s_R=(e_\theta^TJ_R\nu-r_d\Omega_R,\ n_\theta^TJ_R\nu)^T", "3.7")
eq(r"s_C=(e_\psi^TJ_C\nu-r_c\Omega_C,\ n_\psi^TJ_C\nu)^T", "3.8")
p("Kayma tanımı zemin üzerindeki temas hızıdır. Bu nedenle sürtünme kaymaya zıt olmalıdır. İşaret kontrolü: duran gövdede pozitif Ω_L için s_Lx negatiftir; sol tekerin gövdeye aktardığı ileri kuvvet pozitiftir.")

page("4 Düşey destek yüklerinin çözümü")
p("A orijinli gövde koordinatlarında tahrik temasları (0,±b/2), caster teması (c_x,c_y), gövde COM'u (a_x,a_y) olsun. Caster toplulaştırılmış kütlesinin yatay COM'u C'dedir.")
eq(r"c_x=p_x-l\cos\phi,\qquad c_y=p_y-l\sin\phi", "4.1")
eq(r"N_L+N_R+N_C=(m_b+m_c)g", "4.2")
eq(r"c_xN_C=g(m_ba_x+m_cc_x)", "4.3")
eq(r"\frac{b}{2}(N_L-N_R)+c_yN_C=g(m_ba_y+m_cc_y)", "4.4")
p("İlk moment denkleminden N_C'yi; ikinci moment denkleminden farkı; toplamdan iki tahrik yükünü bul:")
eq(r"N_C=\frac{g(m_ba_x+m_cc_x)}{c_x},\quad \Delta N=\frac{2g(m_ba_y+m_cc_y)-2c_yN_C}{b}", "4.5")
eq(r"N_L=\frac{(m_b+m_c)g-N_C+\Delta N}{2},\quad N_R=\frac{(m_b+m_c)g-N_C-\Delta N}{2}", "4.6")
p("Koşul: c_x≠0 ve tüm N değerleri pozitif. Kod negatif yükü sıfıra kırpmaz; destek dışına çıkan parametreleri reddeder. Bu yaklaşım devrilme anını simüle etmez.")
p("Kontrol hesabı, φ=0: c_x=-0.18-0.05=-0.23 m, c_y=0. Toplam ağırlık 16.6×9.81=162.846 N. N_C=9.81×(16×(-0.08)+0.6×(-0.23))/(-0.23)=60.482087 N. Simetriden N_L=N_R=51.181957 N.")

page("5 Kinetik enerji ve kütle matrisi")
eq(r"T=\frac{1}{2}m_b v_B^Tv_B+\frac{1}{2}I_b\dot\theta^2+\frac{1}{2}m_c\Vert J_C\nu\Vert^2", "5.1a")
eq(r"\phantom{T=}+\frac{1}{2}I_s(\dot\theta+\dot\phi)^2+\frac{1}{2}I_d(\Omega_L^2+\Omega_R^2)+\frac{1}{2}I_w\Omega_C^2", "5.1b")
eq(r"D_0=\mathrm{diag}(m_b,m_b,I_b,0,I_d,I_d,I_w),\quad h=(0,0,1,1,0,0,0)^T", "5.2")
eq(r"M=D_0+m_cJ_C^TJ_C+I_s hh^T,\qquad T=\frac{1}{2}\nu^TM\nu", "5.3")
p("<b>Önerme:</b> pozitif kütle ve ataletlerde M simetrik pozitif tanımlıdır. Her terim simetriktir. Bir ν için νᵀMν=0 varsay: m_b terimi v_B=0; I_b terimi θ̇=0; I_s terimi θ̇+φ̇=0 olduğundan φ̇=0; roll ataletleri tüm Ω'ları sıfır yapar. Böylece ν=0'dır. Dolayısıyla sıfır olmayan ν için νᵀMν&gt;0. Geometriye özel bir açı seçimine ihtiyaç yoktur.")
p("Sonuç: geçerli parametrelerde M terslenebilirdir, bağlı ivmelerin çözümü tektir. Kod açık ters matris hesaplamak yerine doğrusal denklem çözer. Bu ispat, sayısal matrisin kötü koşullanamayacağını veya gerçek robot ataletlerinin doğru olduğunu söylemez.")
p("Elle kurmak için: J_C'nin her sütununu bir vektör olarak yaz. (i,j) girdisine m_c çarpı i ve j sütunlarının iç çarpımını ekle. Sonra D₀ köşegenini ve θ/φ bloğunun dört girdisine I_s'yi ekle. 16. sayfada tam sayısal matris verildi.")

page("6 Hareket denklemi ve sanal iş")
p("C'nin hızını tekrar türet. İvme; ivme bilinmeyenleriyle doğrusal J_C ν̇ terimi ve merkezcil c terimine ayrılır:")
eq(r"a_C=J_C\dot\nu+c,\quad c=-d\dot\theta^2+l e_\psi(\dot\theta+\dot\phi)^2", "6.1")
eq(r"M\dot\nu+b_q=Q,\qquad b_q=m_cJ_C^Tc", "6.2")
p("b_q adı kodda 'bias'tır; tahrik iz genişliği b ile karıştırma. Kütle merkezlerinin Newton denklemlerini ve sabit ataletli dönme denklemlerini Jᵀ ile genelleştirmek (6.2)'yi verir. Eşdeğer olarak T'den Euler-Lagrange uygulanır. Ataletler sabittir; caster translasyonunun konuma bağlılığı J ve c ile taşınır.")
eq(r"A_L=J_L-r_d e_\theta e_5^T,\quad A_R=J_R-r_d e_\theta e_6^T,\quad A_C=J_C-r_c e_\psi e_7^T", "6.3")
p("e_i burada 7-boyutlu standart baz vektörüdür, i sayımı 1'den başlar. Aν dünya çerçevesindeki temas kaymasıdır. Yerel kuvvetleri ilgili (e,n) bazında dünyaya çevir.")
eq(r"Q=A_L^TF_L+A_R^TF_R+A_C^TF_C+e_5(\tau_L-\tau_{rL})+e_6(\tau_R-\tau_{rR})", "6.4a")
eq(r"\phantom{Q=}-e_7\tau_{rC}-e_4\tau_{\mathrm{bearing}}-(e_3+e_4)\tau_{\mathrm{bore}}", "6.4b")
p("Bearing momenti iki cisim arasındadır: göreli φ denklemine girer. Zeminden gelen bore momenti ψ=θ+φ açısına etkir: hem θ hem φ denklemine girer. Bu ayrım enerji dengesinin doğru kapanması için gereklidir.")
eq(r"\dot\nu=\mathrm{solve}(M,Q-b_q)", "6.5")

page("7 Pasif temas ve sürtünme sınırı")
p("Tahrik tekeri için k_x,k_y&gt;0, L=μN≥0, z=(k_x s_x,k_y s_y) tanımla:")
eq(r"F_{\mathrm{local}}=-\frac{Lz}{\sqrt{L^2+z^Tz}},\qquad \Vert F\Vert\leq L", "7.1")
eq(r"-F^Ts=\frac{L(k_xs_x^2+k_ys_y^2)}{\sqrt{L^2+z^Tz}}\geq0", "7.2")
p("L=0 olduğunda kuvvet sıfır tanımlanır. Caster'ın kuvvet ve bore momenti ayrı ayrı bütçeyi tüketmesin diye üç bileşenli ortak sınır kullanılır. r_b&gt;0 için:")
eq(r"L_c=\mu_cN_C,\quad z_c=(k_{cx}s_{cx},k_{cy}s_{cy},L_c\dot\psi/\epsilon_b)^T,\quad a_c=\frac{L_c}{\sqrt{L_c^2+z_c^Tz_c}}", "7.3")
eq(r"F_{c,\mathrm{local}}=-a_c(z_{c1},z_{c2})^T,\qquad \tau_{\mathrm{bore}}=a_cz_{c3}r_b", "7.4")
eq(r"\sqrt{\Vert F_c\Vert^2+(\tau_{\mathrm{bore}}/r_b)^2}=\frac{L_c\Vert z_c\Vert}{\sqrt{L_c^2+\Vert z_c\Vert^2}}\leq L_c", "7.5")
eq(r"-F_c^Ts_c+\tau_{\mathrm{bore}}\dot\psi=a_c(k_{cx}s_{cx}^2+k_{cy}s_{cy}^2+r_bL_c\dot\psi^2/\epsilon_b)\geq0", "7.6")
eq(r"\tau_r=\mu_rNr\tanh(\Omega/\epsilon_r)+b_r\Omega,\quad \tau_{\mathrm{bearing}}=b_s\dot\phi+T_s\tanh(\dot\phi/\epsilon_s)", "7.7")
p("x tanh(x/ε)≥0 nedeniyle her direnç momentinin hızla çarpımı sıfırdan küçülemez. Bu yasa tam stiction, lastik elastisitesi veya ölçülmüş malzeme yasası değildir. r_b=0 durumunda üçüncü bileşen ve bore momenti sıfır tanımlanır.")

page("8 Enerji özdeşliğinin ispatı")
p("J_C'nin zaman türevi ile ν çarpımı c'dir. Dolayısıyla νᵀb_q=m_c(J_Cν)ᵀ(J̇_Cν)=½νᵀṀν. T'yi türet ve hareket denklemine (6.2) yerleştir:")
eq(r"\dot T=\nu^TM\dot\nu+\frac{1}{2}\nu^T\dot M\nu=\nu^T(M\dot\nu+b_q)=\nu^TQ", "8.1")
p("Sanal işte A_iν temas kaymasıdır. Bu yüzden νᵀA_iᵀF_i=F_iᵀs_i; yerel/dünya rotasyonu iç çarpımı değiştirmez.")
eq(r"P_m=\tau_L\Omega_L+\tau_R\Omega_R", "8.2")
eq(r"P_d=-F_L^Ts_L-F_R^Ts_R-F_C^Ts_C+\tau_{rL}\Omega_L+\tau_{rR}\Omega_R+\tau_{rC}\Omega_C", "8.3a")
eq(r"\phantom{P_d=}+\tau_{\mathrm{bearing}}\dot\phi+\tau_{\mathrm{bore}}(\dot\theta+\dot\phi)\geq0", "8.3b")
eq(r"\dot T=P_m-P_d,\quad \dot W=P_m,\quad \dot D=P_d,\quad E=T-T_0+D-W=0", "8.4")
p("Bu, seçilen sürekli mekanik modelin cebirsel enerji özdeşliğidir. Sonlu adımlı RK4 için E tam sıfır olmaz; çözüm hatasını ölçer. Motor torkunun kontrolcü tarafından seçilmesi özdeşliği değiştirmez. Konservatif kontrol deneyinde tüm kayıplar ve torklar kapalıysa T sabittir.")
p("Bu eşitlik, enerji kontrolünün neden bir 'çıktı süsü' olmadığını da gösterir: kuvvet/moment işareti veya θ/φ moment dağıtımı yanlışsa denge bozulur. Öte yandan enerji dengesinin tutması, tek başına modelin tüm fiziksel ayrıntılarının gerçek olduğunu kanıtlamaz.")

page("9 Önceki kontrol neden bitiremiyordu")
p("Eski dış kontrol v_c=v_r+e_x, ω_c=ω_r+2e_θ+3v_r e_y idi. Hedef durduğunda v_r=ω_r=0 olur. e_x=e_θ=0 fakat e_y≠0 seçilirse iki komut da sıfırdır. Duran mekanik model sıfır torkta dengededir; yanal hata kalıcı bir karşı örnektir.")
eq(r"v_r=\omega_r=e_x=e_\theta=0,\quad e_y=0.03\ \mathrm{m}\quad\Longrightarrow\quad v_c=\omega_c=0", "9.1")
p("İkinci sorun yalnız P tekerlek servosudur. Basit sabit yük d altında J Ω̇=τ-d, τ=K_p(Ω*−Ω). Denge hızı:")
eq(r"e_\Omega^*=\frac{d}{K_p}\neq0", "9.2")
p("Bu eşitlik gerçek caster yükünün sabit olduğunu varsaymaz; P servonun yük altında neden kayıcı hata gerektirdiğini açıklayan alt sistem karşı örneğidir. Ayrıca bir saniyelik son gözlem her manevranın düzelmesine yetmiyordu.")
table([["Eski kontrol, yüksek dirençli sentetik zemin", "Hedef bitiminden 8 s sonra"],
       ["Düz gidiş", "Konum hatası ≈11.18 mm"], ["Dönüş", "Konum hatası ≈36.80 mm; yanal bileşen ≈−35.80 mm"]], [300,178])
p("Öyleyse yalnız gözlem penceresini uzatmak dönüş hatasını gidermez. V2.1 düzeltmesi: doğru takip hata geometrisi, anti-windup'lı PI tekerlek kontrolü ve duruş hedefinde ayrı konum/açı düzenlemesi. Eski kod ve sonuçlar arşivlenmiştir.")

page("10 Referans yol ve hata dinamiği")
p("Bir segmentte mesafe L_s, yaw değişimi Δθ ve süre T_s tanımlıdır. Düz segment x=L_s s(u), y=0'dır. Eğri segmentte R_s=L_s/Δθ ile yerel yer değiştirme (R_s sin(Δθs), R_s[1−cos(Δθs)]) olur; başlangıç pozu ile dünya çerçevesine taşınır.")
eq(r"u=t/T_s,\quad s_b(u)=u,\quad s_s(u)=10u^3-15u^4+6u^5", "10.1")
eq(r"s_s'(u)=30u^2(1-u)^2,\quad s_s''(u)=60u(1-u)(1-2u),\quad \max s_s'=15/8", "10.2")
p("s_s(0)=0, s_s(1)=1; uçlarda ilk ve ikinci türev sıfırdır. Böylece hız ve ivme uçlarda sıfırdır; jerk uçta genellikle sıfır değildir. Aynı tepe hız için T_s,smooth=(15/8)T_s,baseline kullanılır.")
eq(r"v_r=L_s\dot s,\qquad \omega_r=\Delta\theta\dot s", "10.3")
eq(r"(e_x,e_y)^T=R(\theta)^T(p_r-p_A),\qquad e_\theta=\theta_r-\theta", "10.4")
p("Aşağıdaki türetimde ideal aks kinematiği ṗ_A=v(cosθ,sinθ)ᵀ, θ̇=ω kabul edilir; θ hata dalı süreklidir. Dönen bazın türevi Ṙᵀ=−ω S Rᵀ olduğundan:")
eq(r"\dot e_x=\omega e_y-v+v_r\cos e_\theta,\quad \dot e_y=-\omega e_x+v_r\sin e_\theta,\quad \dot e_\theta=\omega_r-\omega", "10.5")
p("Kod hata açısını atan2(sin,cos) ile sarar. ±π dal kesimi ve doygunluk noktaları küresel türev ispatına dahil değildir.")

page("11 Takip kontrolünün Lyapunov hesabı")
eq(r"v_c=v_r\cos e_\theta+k_x e_x,\quad \omega_c=\omega_r+k_yv_r e_y+k_\theta\sin e_\theta", "11.1")
eq(r"k_x=2\ \mathrm{s}^{-1},\quad k_y=6\ \mathrm{m}^{-2},\quad k_\theta=3\ \mathrm{s}^{-1}", "11.2")
eq(r"V=\frac{1}{2}(e_x^2+e_y^2)+\frac{1-\cos e_\theta}{k_y}\geq0", "11.3")
p("Doygunluk yokken v=v_c, ω=ω_c varsay. (10.5)'i V̇'ye koy:")
eq(r"\dot V=e_x(\omega e_y-v+v_r\cos e_\theta)+e_y(-\omega e_x+v_r\sin e_\theta)+\frac{\sin e_\theta}{k_y}(\omega_r-\omega)", "11.4")
eq(r"\dot V=-k_xe_x^2+v_re_y\sin e_\theta-\frac{\sin e_\theta}{k_y}(k_yv_re_y+k_\theta\sin e_\theta)", "11.5")
eq(r"\dot V=-k_xe_x^2-\frac{k_\theta}{k_y}\sin^2 e_\theta\leq0", "11.6")
p("İleri ve geri v_r işaretleri çapraz terimlerde iptal olur. Ancak bu yarı-negatif türev tek başına e_y→0 demek değildir. v_r=0 iken yanal hata için uyarım yoktur; sonlu bir hareketten sonra duruş stabilizasyonu ayrı ele alınmalıdır.")
p("Tam M2 ideal kaymasız unicycle değildir. Gerçek aks boyuna hızı v=v_c+δv, yaw hızı ω=ω_c+δω ve yanal hızı v_⊥ olsun. Aynı cebir:")
eq(r"\dot V=-k_xe_x^2-\frac{k_\theta}{k_y}\sin^2e_\theta-e_x\delta v-e_yv_\perp-\frac{\sin e_\theta}{k_y}\delta\omega", "11.7")
p("Son üç terim kayma, servo hatası ve doygunluk etkilerini açıkça gösterir. İşaretleri her durumda negatif değildir. Bu nedenle ideal ispatı tam M2 için küresel kararlılık diye sunmuyoruz.")

page("12 Duruş hedefinde konum yaklaşması")
p("Hareket profili bitince hedef poz sabittir. ρ=‖p_g−p_A‖ ve β=atan2(y_g−y_A,x_g−x_A) olsun. İlk girişte s=+1 veya −1 seçilerek hedef yönüne en yakın ileri/geri yaklaşım belirlenir ve o yaklaşım boyunca sabit tutulur.")
eq(r"\theta_e=\theta+\frac{1-s}{2}\pi,\quad \alpha=\mathrm{wrap}(\beta-\theta_e),\quad |\alpha(0)|\leq\pi/2", "12.1")
eq(r"v_c=s k_\rho\rho\cos\alpha,\quad \omega_c=k_\alpha\alpha+k_\rho\sin\alpha\cos\alpha,\quad k_\rho=2,\ k_\alpha=3", "12.2")
p("İdeal, doygunluksuz aks kinematiğinde ve ρ&gt;0 için etkin ileri hız u=s v'dir. Polar türevler:")
eq(r"\dot\rho=-u\cos\alpha,\quad \dot\beta=(u/\rho)\sin\alpha,\quad \dot\alpha=(u/\rho)\sin\alpha-\omega", "12.3")
eq(r"\dot\rho=-k_\rho\rho\cos^2\alpha,\qquad\dot\alpha=-k_\alpha\alpha", "12.4")
p("Böylece α(t)=α₀e^(−k_αt), ve ρ(t)=ρ₀ exp[−k_ρ∫cos²α dt]. Sonlu bir t₁'den sonra |α|≤π/4 olduğundan cos²α≥1/2; dolayısıyla ρ(t)≤ρ(t₁)exp[−k_ρ(t−t₁)/2]→0. Bu gerçek bir nominal konum yakınsama ispatıdır; hedef yaw henüz düzenlenmemiştir.")
eq(r"\frac{d}{dt}\left(\frac{1}{2}\rho^2+\frac{\ell_0^2}{2}\alpha^2\right)=-k_\rho\rho^2\cos^2\alpha-\ell_0^2 k_\alpha\alpha^2\leq0", "12.5")
p("ℓ₀&gt;0 sabit uzunluk ölçeğidir; iki terimin boyutunu eşler. Kodda ρ≤6 mm olunca açı hizalamasına geçilir. Yer değiştirme yeniden 12 mm'yi aşarsa konum yaklaşması yeniden başlar. Bu histerezis, tek eşik çevresindeki mod titreşimini azaltır.")
p("Uygulamada |v_c|≤0.25 m/s, |ω_c|≤1 rad/s sınırı vardır. (12.4) doygunluk sırasında aynen geçerli değildir; mod geçişli, doygun ve kaymalı tam sistem için ayrıca küresel ispat ileri sürülmez.")

page("13 Son açı ve tamamlanma ölçütü")
eq(r"e_g=\mathrm{wrap}(\theta_g-\theta),\qquad v_c=0,\qquad \omega_c=k_g\sin e_g,\quad k_g=3", "13.1")
eq(r"V_g=1-\cos e_g,\quad \dot V_g=\sin e_g\dot e_g=-k_g\sin^2 e_g\leq0", "13.2")
p("İdeal yerinde dönme altında konum değişmez. |e_g|&lt;π dalında e_g=0 yerel asimptotik kararlıdır; e_g=π antipodal dengesi dışarıda tutulur. Gerçek M2'de caster'ın dönüş kayması konumu kaydırabilir; bu nedenle 12 mm geri-dönüş eşiği vardır. Histerezis ve ideal alt-mod ispatları tek başına tam hibrit sistemin kararlılık ispatını oluşturmaz.")
table([["Dış tamamlanma koşulu", "Eşik"], ["Aks konum hatası", "≤0.020 m"], ["Sarılmış yaw hatası", "≤0.050 rad"],
       ["Düzlemde toplam hız √(v_long²+v_lat²)", "≤0.020 m/s"], ["Mutlak yaw hızı", "≤0.050 rad/s"],
       ["Tüm koşulların kesintisiz sağlanması", "≥0.250 s; zaman damgalarıyla"],
       ["Denetim başlangıcı", "Planlanan hareket profili sonu"]], [330,148])
p("Konum/açı/yaw hızı eşikleri önceki sürümle aynıdır. Eski sürüm yalnız boyuna hızı ölçüyordu; yeni sürüm toplam düzlemsel hızı denetleyerek koşulu sıkılaştırır. 26 örnek saymak yerine gerçek 0.25 s süre kontrol edilir.")
p("Tamamlama zamanı, 0.25 s kalışın doğrulanmış başlangıcıdır; karar en erken 0.25 s sonra verilebilir. Her koşuda hem eski 'profil sonu+1 s' penceresi hem yeni 'profil sonu+8 s' toparlanma penceresi raporlanır. Hareket profil süresi ile görev tamamlama süresi aynı şey değildir.")

page("14 Tekerlek PI ve yük reddi")
eq(r"\Omega_L^*=(v_c-b\omega_c/2)/r_d,\qquad \Omega_R^*=(v_c+b\omega_c/2)/r_d", "14.1")
p("Her teker için e=Ω*−Ω, K_p=0.15 N m s/rad ve K_i=0.8 N m/rad. Zaman adımında saklanan I_t bir torktur, hata integrali değildir:")
eq(r"I_{\mathrm{cand}}=I_t+K_i\Delta t\,e,\quad \tau_{\mathrm{raw}}=K_pe+I_{\mathrm{cand}},\quad \tau_s=\mathrm{clip}(\tau_{\mathrm{raw}},-0.5,0.5)", "14.2")
p("|τ_raw|≤0.5 veya e(τ_raw−τ_s)≤0 ise I_t←I_cand; aksi halde eski I_t tutulur. Uygulanan tork clip(K_p e+I_t, ±0.5)'tir. Bu koşullu integrasyon, doygunluğu artıran hatanın birikmesini önler; tork her durumda sınırlıdır.")
p("<b>Nominal alt sistem ispatı.</b> JΩ̇=τ−d, sabit Ω*, sabit yük d, J&gt;0 ve doygunluk yok varsay. Ĩ=I_t−d ile:")
eq(r"J\dot e=-K_pe-\widetilde I,\qquad \dot{\widetilde I}=K_i e", "14.3")
eq(r"V_i=\frac{1}{2}Je^2+\frac{\widetilde I^2}{2K_i},\quad \dot V_i=e(-K_pe-\widetilde I)+\widetilde I e=-K_pe^2\leq0", "14.4")
p("V̇_i=0 kümesinde e=0; bu kümede kalmak için ė=−Ĩ/J=0, yani Ĩ=0 gerekir. En büyük değişmez küme (0,0)'dır; sabit yükte e→0 ve I_t→d. Karakteristik polinom Jλ²+K_pλ+K_i'nin katsayıları pozitiftir. Bu alt sistemin kararlılığı, değişken yük/tekerlek-kuvvet bağlaşımı içeren tam M2'ye otomatik taşınmaz.")
p("<b>Örneklemeli nominal PI.</b> Sabit yük ve aralık boyunca sabit tork altında, önce integrali güncelleyen (14.2) tam olarak aşağıdaki ayrık doğrusal sistemi verir. Burada a=hK_p/J, b_i=h²K_i/J, c=h/J'dir; b_i mekanik bias değildir.")
eq(r"e_{k+1}=(1-a-b_i)e_k-c\widetilde I_{k-1},\qquad \widetilde I_k=K_i h e_k+\widetilde I_{k-1}", "14.5")
eq(r"\mathrm{tr}(A)=2-a-b_i,\quad\det(A)=1-a,\quad a>0,\ b_i>0,\ 4-2a-b_i>0", "14.6")
p("Son üç koşul ikinci dereceli Jury ölçütünün 1−det&gt;0 ve 1±tr+det&gt;0 eşitsizlikleridir. h=0.01 s, J=0.0045 için a=1/3, b_i≈0.017778; 4−2a−b_i≈3.315556&gt;0. Dolayısıyla bu sabit-yüklü, doygunluksuz örneklemeli alt sistemin özdeğerleri birim çember içindedir. Tam M2 bu tek tekerlik alt sistem değildir.")
p("Elle kontrol: J=0.0045, d=(0.08,0.10) N m, Ω*=(2,2) rad/s. Eski P denge hatası (0.533333,0.666667) rad/s olur. PI testinde 0.01 s adımla 4 s sonra hızlar 2 rad/s'ye, integral torkları bu iki yüke 10⁻⁵ tolerans içinde yaklaşır.")

page("15 Elle çözüm için sayısal başlangıç")
table([["Parametre", "Değer"], ["m_b, m_c; g", "16 kg, 0.6 kg; 9.81 m/s²"], ["I_b, I_s, I_d, I_w", "0.45; 0.0015; 0.0045; 0.00045 kg m²"],
       ["a; p; b; l", "(−0.08,0) m; (−0.18,0) m; 0.38 m; 0.05 m"], ["r_d; r_c", "0.075 m; 0.040 m"],
       ["μ_d, μ_c; k_dx,k_dy; k_cx,k_cy", "0.6,0.6; 300,600; 200,250 [N s/m]"],
       ["μ_rd, μ_rc; b_rd,b_rc", "0.008,0.012; 0.00025,0.00010 [N m s/rad]"],
       ["b_s; T_s; r_b", "0.004 N m s/rad; 0.006 N m; 0.002 m"],
       ["ε_r; ε_s; ε_b", "0.10; 0.08; 0.12 rad/s"]], [200,278])
p("Durum: (x_B,y_B,θ,φ)=(-0.08,0,0.3,0.7); ν=(0.2,-0.1,0.4,-0.8,4,2,3). W=D=0; aks torkları (0.12,-0.03) N m. Tüm trigonometrik hesaplar radyanla.")
table([["Ara büyüklük", "Sonuç"], ["r = p_C−p_B [m]", ", ".join(f"{v:.9f}" for v in sample["r"])],
       ["N_L, N_R, N_C [N]", ", ".join(f"{v:.9f}" for v in sample["normal"])],
       ["s_L [m/s]", ", ".join(f"{v:.9f}" for v in sample["slips"]["left"])],
       ["s_R [m/s]", ", ".join(f"{v:.9f}" for v in sample["slips"]["right"])],
       ["s_C [m/s]", ", ".join(f"{v:.9f}" for v in sample["slips"]["caster"])],
       ["F_L,yerel [N]", ", ".join(f"{v:.9f}" for v in sample["forces"]["left"])],
       ["F_R,yerel [N]", ", ".join(f"{v:.9f}" for v in sample["forces"]["right"])],
       ["F_C,yerel [N]", ", ".join(f"{v:.9f}" for v in sample["forces"]["caster"])]], [170,308])

page("16 Sayısal kütle matrisi ve ivmeler")
numeric_matrix("M · sıra: x_B, y_B, θ, φ, γ_L, γ_R, γ_C", sample["M"])
p("Önceki sayfadaki kuvvetleri dünyaya çevir; (6.4)'te yerine koy. Tablo satırları aynı genelleştirilmiş koordinat sırasındadır. İlk iki satır N, sonraki satırlar N m birimindedir.")
table([["i", "b_q", "Q", "ν̇"], *[[i+1, f"{sample['b'][i]:.9f}", f"{sample['Q'][i]:.9f}", f"{sample['accelerations'][i]:.9f}"] for i in range(7)]], [28,150,150,150])
p("Direnç momentleri (sol roll, sağ roll, caster roll, bearing, bore) [N m]: " + ", ".join(f"{v:.9f}" for v in sample["resistance_torques"]) + ".")
p(f"T={sample['energy']:.9f} J; P_m={sample['motor_power']:.9f} W; P_d={sample['dissipation']:.9f} W. "
  f"Tam hassasiyetle max|Mν̇+b_q−Q|={sample['linear_residual_max']:.3g}. Yuvarlatılmış basılı matrisle son basamaklar farklı çıkar.")
p("Elle çözüm sırası: (1) M ve b_q'yu geometri/ataletlerden kur; (2) kayma→kuvvet→Q zincirini hesapla; (3) yedi bağlı doğrusal denklemi çöz. Bağlı θ ve φ denklemlerini ayrı tek serbestlik dereceli denklemler gibi çözmek yanlış olur.")

page("17 Bir RK4 adımını yeniden üret")
eq(r"f(z)=(v_{Bx},v_{By},\dot\theta,\dot\phi,\dot\nu^T,P_m,P_d)^T", "17.1")
eq(r"k_1=f(z),\quad k_2=f(z+hk_1/2),\quad k_3=f(z+hk_2/2),\quad k_4=f(z+hk_3)", "17.2")
eq(r"z^+=z+\frac{h}{6}(k_1+2k_2+2k_3+k_4),\qquad h=0.0005\ \mathrm{s}", "17.3")
p("Tork bu dört aşamada sabit (0.12,-0.03) N m'dir. Kuvvetleri ve M'yi ara durumlarda yeniden hesapla. Aşağıdaki sütunlar aynı 13-durum sırasındadır; z⁺ birimlerini durum tanımından al.")
table([["Bileşen", "k₁", "z⁺"], *[[name, f"{sample['rk4']['k1'][i]:.9f}", f"{sample['rk4']['next_state'][i]:.9f}"] for i,name in enumerate(["x_B","y_B","θ","φ","v_Bx","v_By","θ̇","φ̇","Ω_L","Ω_R","Ω_C","W","D"])]], [70,204,204])
p("k₂,k₃,k₄ ve tüm ara büyüklüklerin yuvarlatılmamış kayıtları: evidence/hand-verification-example.json. RK4, düzgün ve tek komut aralığındaki bir türev için yerel O(h⁵), global O(h⁴) doğruluğa sahiptir. Komut sıçramalarında adım komut sınırına hizalanır; mod geçişleri ve doygunluklar evrensel dördüncü mertebe iddiasını sınırlar.")

page("18 RK4 ara aşamaları ve kontrol örneği")
p("Aşağıdaki k₂, k₃ ve k₄ sütunlarını (17.2) ile üret. Bileşen sırası önceki sayfayla aynıdır. Aks torku her aşamada aynı tutulur; geometri ve kayma ara durumdan hesaplanır.")
table([["Bileşen", "k₂", "k₃", "k₄"], *[[name, *[f"{sample['rk4'][key][i]:.9f}" for key in ("k2","k3","k4")]] for i,name in enumerate(["x_B","y_B","θ","φ","v_Bx","v_By","θ̇","φ̇","Ω_L","Ω_R","Ω_C","W","D"])]], [58,140,140,140])
p("Takip yasasını ayrı kontrol et: e_x=0.01 m, e_y=−0.005 m, e_θ=0.1 rad; v_r=0.15 m/s, ω_r=π/6 rad/s. Doygunluk öncesi v_c=0.169250625 m/s, ω_c=0.818599026 rad/s. (11.6)'dan V̇=−0.005183356 m²/s bulunur. Bu değerler uygulama sınırlarının içindedir.")
p("Sayısal örneğin basılı son basamaklarında yuvarlama vardır. Kâğıtta yeniden hesaplarken ara değerleri mümkün olduğunca çok basamakla taşı. İvme çözümünü basılı M ile yapıyorsan 9 basamaklı girdiler kullan; tam hassasiyet karşılaştırması için JSON kaydı korunmuştur.")

page("19 Aynı koşullarda önce ve sonra")
p("İki sentetik zemin, dört manevra, iki profil. Aynı başlangıç caster açısı 45°, aynı geometri, aynı temel mekanik parametreler, aynı komut/çözüm adımları ve aynı ±0.5 N m tork sınırı korundu. Kontrol kazançları değişti; yeni kontrol ayrıca terminal düzeltme hareketi yapar.")
rows = [["Zemin / manevra / profil", "Eski bitiş s", "Yeni bitiş s", "Yeni 1 s?", "Son hata mm"]]
for run in nominal:
    old = next(r for r in before["runs"] if r["run_id"] == run["run_id"])
    a, b = old["metrics"]["completion_time_s"], run["metrics"]["completion_time_s"]
    label = ("A" if run["surface"] == "reference_synthetic" else "B") + " / " + run["maneuver"] + " / " + ("std" if run["profile"] == "baseline" else "smooth")
    rows.append([label, "yok" if a is None else f"{a:.2f}", "yok" if b is None else f"{b:.2f}",
                 "evet" if run["metrics"]["completion_within_original_1s_window_s"] is not None else "hayır",
                 f"{1000*run['metrics']['final_position_error_m']:.2f}"])
table(rows,[210,68,68,57,75])
p(f"A referans sentetik; B yüksek dirençli sentetik zemindir. Yeni 1 s sütunu, 0.25 s kalışın tamamının eski pencere içinde doğrulanmasını ister. "
  f"Özet: eski 3/16; yeni aynı pencerede {same_window}/16; yeni 8 s toparlanmada {completed}/16. Duyarlılık dahil {all_completed}/40.", "small")
p("Yeni uzun pencerenin toplam RMSE'sini eski kısa pencereyle doğrudan kıyaslamak yanıltır. study.json içindeki tracking_on_original_window aynı zaman aralığını, tracking_during_profile ise yalnız hareket profilini karşılaştırmaya yarar. Terminal düzeltme jerk ve yol uzunluğunu artırabilir; düzgünlük sonucu ayrıca değerlendirilmelidir.")

page("20 Doğrulama sınırı ve tekrar üretim")
p("<b>Sayısal kontroller:</b> V2.1 tam test kümesi 31/31 geçti. Bunlar önceki 24 kontrole ek olarak lateral kilitlenme, PI sabit yük reddi, iki zeminde dönüş bitişi, süre tabanlı kalış, takip Lyapunov özdeşliği ve terminal polar hata dinamiğini sınar. V1 testi tekrar çalıştırıldı; kaynaklar 60 dosyalık SHA-256 envanteriyle karşılaştırılır.")
p("40 rastgele mekanik durumda M'nin pozitif özdeğerleri, nokta hızlarından enerji ile ½νᵀMν eşitliği, sonlu fark Euler-Lagrange bias'ı ve güç dengesi denetlenir. Bu örneklem denetimleri cebirsel ispatın yerine geçmez; kodlama hatalarını yakalamak içindir.")
p("<b>Kod eşlemesi:</b> mekanik geometri/Jacobian: V1 physics/geometry.py; destek: support_loads.py; M,b_q: inertia.py; temas: contact_forces.py; Q ve ivme: motion.py; enerji: energy.py. V2.1 takip/terminal/PI: twin.py içindeki TrackingController. Durum ilerletme: integrate; tamamlanma: completion_time.")
p("Çalışma klasörü: /Users/tora/Documents/ChatGPT/arche/caster-physics-python-v2", "small")
p("Komutlar (V1 ortamıyla):<br/>../caster-physics-python/.venv/bin/python -m pytest -q tests<br/>../caster-physics-python/.venv/bin/python run_study.py --group all --name yeni-bir-kosum", "small")
p("Sayısal örnek: evidence/hand-verification-example.json.<br/>Önceki 40 koşu: outputs/validation-2026-11-13.<br/>Düzeltilmiş 40 koşu: outputs/tracking-fix-v2.1-2026-11-13.<br/>Eski V2 kaynak arşivi: evidence/v2-before-tracking-fix-2026-11-13.tar.gz.", "small")
p("<b>İspatın kapsamı:</b> sayfa 5-8'de seçilen mekanik modelin kesin cebirsel özellikleri; sayfa 11-14'te ideal alt sistemlerin belirtilen varsayımlar altındaki kontrol özellikleri gösterildi. Doygunluk, örnekleme, kayma, değişken yük ve hibrit geçişler bir araya geldiğinde tam M2 için küresel yakınsama ispatlanmadı. Fiziksel kalibrasyon, sensör gecikmesi, robot yerelleştirmesi ve gerçek robot deneyi yapılmadı.")
p("İlgili klasik çalışma: Kanayama, Kimura, Miyazaki ve Noguchi, 'A stable tracking control method for an autonomous mobile robot', ICRA 1990, DOI 10.1109/ROBOT.1990.126006. Buradaki eşitlikler kullanılan koddan bağımsız cebirle açıkça türetilmiştir; bir literatür teoremi tam caster modeline aktarılmamıştır.", "small")
source_hashes = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ("twin.py","run_study.py")}
for name,digest in source_hashes.items():
    p(f"{name} SHA-256: {digest}", "small")

specification = TEMP / "equations.json"
specification.write_text(json.dumps(equations,ensure_ascii=False))
subprocess.run([str(ROOT.parent/"caster-physics-python/.venv/bin/python"), str(ROOT/"docs/render_math_assets.py"), str(specification), str(TEMP)],check=True)


def decorate(canvas, document):
    canvas.setFillColor(colors.HexColor("#6c7d88"))
    canvas.setFont("Body", 7.2)
    canvas.drawString(58, 811, "ARCHE / ME492     Matematiksel türetim ve doğrulama")
    canvas.drawString(58, 28, "13 Kasım 2026 · V2.1 · SI birimleri")
    canvas.drawRightString(537, 28, str(document.page))


destination = OUTPUT / "Arche_V2_Matematiksel_Turetim_ve_Dogrulama_TR.pdf"
document = SimpleDocTemplate(str(destination), pagesize=(595.28,841.89), rightMargin=58,leftMargin=58,
                             topMargin=56,bottomMargin=48,title="Arche V2 matematiksel türetim ve doğrulama",author="Arche ME492")
document.build(story,onFirstPage=decorate,onLaterPages=decorate)
from pypdf import PdfReader
reader=PdfReader(destination)
assert len(reader.pages)==len(page_titles), (len(reader.pages),len(page_titles))
assert all(len(page.extract_text())>100 for page in reader.pages)
(TEMP/"page-titles.json").write_text(json.dumps(page_titles,ensure_ascii=False,indent=2))
print(f"{destination}\nPages: {len(reader.pages)}; equations: {len(equations)}")
