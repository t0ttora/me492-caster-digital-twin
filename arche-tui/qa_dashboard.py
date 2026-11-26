"""Render the actual terminal drawing calls to PNG for visual layout review."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import json
import dashboard as d

class Screen:
    def __init__(self,cols=125,rows=40):self.w=cols;self.h=rows;self.erase()
    def getmaxyx(self):return self.h,self.w
    def erase(self):self.cells=[[(' ',1<<8) for _ in range(self.w)] for _ in range(self.h)]
    def addnstr(self,y,x,text,n,attr=0):
        assert 0<=y<self.h and 0<=x<self.w
        for i,c in enumerate(str(text)[:n]):
            assert x+i<self.w
            self.cells[y][x+i]=(c,attr)
    def refresh(self):pass
    def contents(self):return '\n'.join(''.join(c for c,a in row) for row in self.cells)

COLORS={1:('#d0d0d0','#262626'),2:('#afff00','#262626'),3:('#eeeeee','#262626'),4:('#af87ff','#262626'),5:('#ff005f','#262626'),6:('#afff00','#262626'),7:('#ffaf00','#262626'),8:('#8a8a8a','#262626'),9:('#000000','#afff00'),10:('#ff005f','#262626')}
def save(screen,path):
    fonts=d.b.ROOT/'caster-physics-python/.venv/lib/python3.12/site-packages/matplotlib/mpl-data/fonts/ttf'
    normal=ImageFont.truetype(str(fonts/'DejaVuSansMono.ttf'),18);bold=ImageFont.truetype(str(fonts/'DejaVuSansMono-Bold.ttf'),18)
    dots=ImageFont.truetype(str(fonts/'DejaVuSans.ttf'),16)
    cw,ch=12,23;im=Image.new('RGB',(screen.w*cw+24,screen.h*ch+24),'#262626');draw=ImageDraw.Draw(im)
    for y,row in enumerate(screen.cells):
        for x,(c,attr) in enumerate(row):
            pair=(attr>>8)&255;fg,bg=COLORS.get(pair,COLORS[1]);xx=x*cw+12;yy=y*ch+12
            draw.rectangle((xx,yy,xx+cw,yy+ch),fill=bg)
            draw.text((xx,yy-1),c,font=dots if 0x2800<=ord(c)<=0x28ff else (bold if attr&d.curses.A_BOLD else normal),fill=fg)
    im.save(path)

if __name__=='__main__':
    out=Path(__file__).resolve().parent/'qa';out.mkdir(exist_ok=True)
    for w,h in [(125,40),(80,24)]:
        screen=Screen(w,h);ui=d.Dashboard(screen);ui.draw();save(screen,out/f'setup-{w}x{h}.png')
        ui.panel='dashboard';ui.draw();save(screen,out/f'dashboard-{w}x{h}.png')
        for panel in ['experiment','experiment_all','geometry','physics','initial','controller','plot','data','archive','help']:
            if panel=='archive':ui.scan()
            ui.panel=panel;ui.focus='content';ui.draw();save(screen,out/f'{panel}-{w}x{h}.png')
    print(out)
