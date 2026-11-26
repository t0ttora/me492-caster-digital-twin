#!/usr/bin/env python3
"""Arche presentation console. Run with the existing V1 Python environment."""
import curses, json, sys, subprocess, time, textwrap, math
from pathlib import Path
from datetime import datetime
import backend as b
import viewer

TABS=['Model','Experiment','Geometry','Physics','Initial state','Controller','Results','Archive']
CHOICES={'preset':[x.name for x in b.EXPERIMENTS],'torque_mode':['preset','constant'],'maneuver':list(b.twin.MANEUVERS),'profile':list(b.twin.PROFILES),'surface':list(b.twin.SURFACES),'design':list(b.twin.DESIGNS),'mode':['auto','manual']}
GROUP={1:'experiment',2:'geometry',3:'dynamics',4:'initial',5:'gains'}

def fmt(v):return json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list,bool)) else str(v)
def flat(d,prefix=''):
    result=[]
    for k,v in d.items():
        if isinstance(v,dict):result+=flat(v,prefix+k+'.')
        else:result.append(prefix+k+' = '+fmt(v))
    return result

class Console:
    def __init__(self,s):
        self.s=s;self.configs={v:b.defaults(v) for v in b.VERSIONS};self.version='V2';self.tab=0;self.index=0;self.offset=0
        self.rows=[];self.other=[];self.source=None;self.metrics={};self.channel='caster_angle_body_rad';self.x='time_s';self.view='plot';self.message='Choose V1 or V2. All model inputs use SI units.';self.job=None;self.log=None;self.started=0;self.archive=[]
    @property
    def c(self):return self.configs[self.version]
    def write(self,y,x,text,style=0):
        h,w=self.s.getmaxyx()
        if 0<=y<h and x<w:
            try:self.s.addnstr(y,max(0,x),str(text),max(0,w-x-1),style)
            except curses.error:pass
    def wrap(self,y,x,text,width,limit=99,style=0):
        for i,line in enumerate(textwrap.wrap(str(text),max(10,width))[:limit]):self.write(y+i,x,line,style)
    def prompt(self,title,value=''):
        text=str(value);cursor=len(text);self.s.timeout(-1);curses.curs_set(1)
        while True:
            h,w=self.s.getmaxyx();self.write(h-3,1,' '*(w-2));self.write(h-3,1,title,curses.A_BOLD);self.write(h-2,1,' '*(w-2));shown=text[max(0,cursor-w+6):];self.write(h-2,1,shown)
            try:self.s.move(h-2,min(w-3,1+min(cursor,w-6)))
            except curses.error:pass
            key=self.s.get_wch()
            if key in ['\n','\r']:break
            if key=='\x1b':text=None;break
            if key in [curses.KEY_BACKSPACE,'\x7f','\b']:
                text=text[:max(0,cursor-1)]+text[cursor:];cursor=max(0,cursor-1)
            elif key==curses.KEY_LEFT:cursor=max(0,cursor-1)
            elif key==curses.KEY_RIGHT:cursor=min(len(text),cursor+1)
            elif key=='\x15':text='';cursor=0
            elif isinstance(key,str) and key.isprintable():text=text[:cursor]+key+text[cursor:];cursor+=1
        curses.curs_set(0);self.s.timeout(100);return text
    def field_items(self):
        group=GROUP.get(self.tab)
        if not group:return []
        if self.tab==5 and self.version=='V2' and self.c['gains']['mode']=='auto':
            try:self.c['gains'].update(b.active_gains(self.c))
            except (ValueError,ZeroDivisionError,TypeError) as error:self.message='Invalid gain inputs: '+str(error)
        if self.tab==5 and self.version=='V1':return []
        return [(k,v) for k,v in self.c[group].items() if k not in ['motor_work_J','dissipated_energy_J']]
    def edit(self):
        items=self.field_items()
        if not items:return
        key,value=items[self.index%len(items)];choices=CHOICES.get(key)
        if choices:
            try:new=choices[(choices.index(value)+1)%len(choices)]
            except ValueError:new=choices[0]
            if key in ['preset','surface','design']:b.select_preset(self.c,key,new)
            else:self.c[GROUP[self.tab]][key]=new
        else:
            entry=self.prompt(key+' | JSON number or [x,y,z]; Ctrl-U clears; Esc cancels',fmt(value))
            if entry is None:return
            new=json.loads(entry)
            if isinstance(value,list):
                if not isinstance(new,list) or len(new)!=len(value):raise ValueError('Enter a vector with '+str(len(value))+' values.')
                if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in new):raise ValueError('Use finite numeric vector components.')
            elif isinstance(new,bool) or not isinstance(new,(int,float)) or not math.isfinite(new):raise ValueError('Enter a finite numeric value.')
            self.c[GROUP[self.tab]][key]=new
            if self.tab==5:self.c['gains']['mode']='manual'
        self.message='Updated '+key+'. R validates the whole configuration before running.'
    def start(self):
        if self.job:raise ValueError('A simulation is already running. X cancels it.')
        _,duration=b.validate(self.c)
        b.OUTPUT.mkdir(parents=True,exist_ok=True);out=b.OUTPUT/(datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'-'+self.version);out.mkdir()
        request=out/'request.json';request.write_text(json.dumps(self.c,indent=2,allow_nan=False))
        self.log=(out/'run.log').open('w');self.job=subprocess.Popen([sys.executable,str(Path(b.__file__)),str(request),str(out)],stdout=self.log,stderr=subprocess.STDOUT)
        self.run_dir=out;self.started=time.monotonic();self.message=f'Running {self.version}: {duration:g} simulated seconds. UI stays responsive; X cancels.'
    def poll(self):
        if self.job and self.job.poll() is not None:
            code=self.job.returncode;self.log.close();self.log=None;self.job=None
            if code==0:
                self.load(self.run_dir/'results.csv');self.tab=6;self.view='metrics';self.offset=0;self.message='Completed. P plot, D data, K metrics, O interactive explorer, G PNG.'
            else:self.message='Run failed. J opens the run log. '+(self.run_dir/'run.log').read_text()[-190:].replace('\n',' ')
    def load(self,path,comparison=False):
        rows=b.read_rows(path)
        if comparison:self.other=rows;self.message='Comparison loaded: '+str(path);return
        self.rows=rows;self.source=Path(path);self.metrics={}
        summary=Path(path).parent/'summary.json'
        if summary.exists():self.metrics=json.loads(summary.read_text())
        else:
            study=Path(path).parent/'study.json'
            if study.exists():
                for run in json.loads(study.read_text()).get('runs',[]):
                    if Path(run.get('run_id','')).name==Path(path).stem:self.metrics=run.get('metrics',{});break
        if not self.metrics:self.metrics={'samples':len(rows),'end_time_s':rows[-1]['time_s'],'final_x_m':rows[-1].get('x_m'),'final_y_m':rows[-1].get('y_m'),'energy_balance_max_J':max(abs(r.get('energy_balance_error_J',0)) for r in rows)}
        if self.channel not in rows[0]:self.channel=next(k for k in rows[0] if k!='time_s')
        self.tab=6;self.offset=0;self.message='Loaded '+str(path)
    def scan(self):
        roots=[b.OUTPUT,b.ROOT/'caster-physics-python/outputs',b.ROOT/'caster-physics-python-v2/outputs']
        self.archive=sorted([p for root in roots if root.exists() for p in root.rglob('*.csv') if 'command' not in p.name.lower()],key=lambda p:p.stat().st_mtime,reverse=True)
    def open_file(self,path):
        command=['open',str(path)] if sys.platform=='darwin' else ['xdg-open',str(path)]
        subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);self.message='Opened '+str(path)
    def draw(self):
        self.s.erase();h,w=self.s.getmaxyx();self.write(0,2,'ARCHE / CASTER LAB   |   '+self.version+' selected   |   SYNTHETIC',curses.A_BOLD|curses.color_pair(1))
        names=TABS if w>=110 else ['Model','Setup','Shape','Physics','State','Gains','Results','Archive']
        self.write(1,2,'  '.join(('['+t+']') if i==self.tab else t for i,t in enumerate(names)),curses.color_pair(1))
        self.write(2,2,'Tab/Shift-Tab pages  |  arrows navigate  |  Enter edit  |  R run  |  S save  |  L load  |  Q quit')
        if h<20 or w<70:self.write(5,2,'Please enlarge the terminal to at least 70 columns x 20 rows.');self.s.refresh();return
        if self.tab==0:
            self.write(5,4,'1  V1 - MECHANICAL BASELINE',curses.A_BOLD);self.wrap(7,4,b.VERSIONS['V1'],w-10,3)
            self.write(11,4,'2  V2 - TRAJECTORY / CONTROL',curses.A_BOLD);self.wrap(13,4,b.VERSIONS['V2'],w-10,3)
            self.write(17,4,'Press 1 or 2, then Tab to edit parameters. A opens previous runs.')
        elif self.tab in GROUP:
            items=self.field_items();visible=h-12
            if not items:self.wrap(6,3,'V1 has no trajectory controller. Its input is wheel axle torque. Select the torque program in Experiment; V2 adds the controller.',w-7,4)
            else:
                self.index%=len(items);begin=max(0,min(self.index-visible//2,len(items)-visible));begin=max(0,begin)
                self.write(4,2, f'{TABS[self.tab]} | {len(items)} fields. Enter cycles a preset or edits a number/vector.',curses.A_BOLD)
                for n,(key,value) in enumerate(items[begin:begin+visible],begin):self.write(6+n-begin,3,('> ' if n==self.index else '  ')+f'{key:<39} {fmt(value)}',curses.A_REVERSE if n==self.index else 0)
                key=items[self.index][0];self.wrap(h-6,3,b.help_for(key),w-7,3,curses.color_pair(2))
            if self.tab==5:self.write(h-7,3,'Built-in V2 limits: +/-0.5 Nm; 0.25 m/s; 1 rad/s. Terminal switching: 6 / 12 mm.',curses.color_pair(2))
        elif self.tab==7:
            self.write(4,2,f'ARCHIVE | {len(self.archive)} CSVs | Enter inspect; C use as comparison; / filter',curses.A_BOLD)
            if self.archive:
                self.index%=len(self.archive);count=h-10;begin=max(0,self.index-count//2)
                for n,p in enumerate(self.archive[begin:begin+count],begin):
                    label=str(p.relative_to(b.ROOT));self.write(6+n-begin,2,('> ' if n==self.index else '  ')+label[-(w-7):],curses.A_REVERSE if n==self.index else 0)
        elif self.tab==6:
            if not self.rows:self.wrap(6,3,'No result selected. R runs the current configuration; A opens existing Python CSVs.',w-8,3)
            else:
                self.write(4,2,f'RESULTS / {self.view.upper()} | P plot  D data  K metrics  [ ] channel  T trajectory  O HTML  G PNG',curses.A_BOLD)
                self.write(5,2,str(self.source)[-(w-5):]);self.write(6,2,f'{len(self.rows)} samples | {self.x} -> {self.channel} | / select channel | C clear comparison')
                if self.view=='metrics':
                    lines=flat(self.metrics) or ['No standalone summary in this archive; all CSV columns remain available.']
                    self.offset=min(self.offset,len(lines)-1)
                    for i,line in enumerate(lines[self.offset:self.offset+h-14]):self.write(8+i,3,line,curses.A_REVERSE if i==0 else 0)
                    key=lines[min(self.offset,len(lines)-1)].split(' = ')[0]
                    self.wrap(h-6,2,b.result_help(key),w-5,3,curses.color_pair(2))
                elif self.view=='data':
                    self.offset=min(self.offset,max(0,len(self.rows)-(h-15)))
                    keys=list(self.rows[0]);j=keys.index(self.channel);cols=['time_s']+list(dict.fromkeys(keys[max(1,j-1):j+3]));cols=cols[:max(2,(w-4)//23)]
                    self.write(8,2,' '.join(f'{k[:21]:>22}' for k in cols),curses.A_BOLD)
                    for i,row in enumerate(self.rows[self.offset:self.offset+h-15]):self.write(9+i,2,' '.join(f'{row[k]:22.9g}' for k in cols))
                else:
                    xs=[r[self.x] for r in self.rows];ys=[r[self.channel] for r in self.rows];lines=b.ascii_plot(xs,ys,w-7,h-15)
                    for i,line in enumerate(lines):self.write(9+i,4,line,curses.color_pair(1))
                    self.write(8,2,f'Y: {min(ys):.6g} .. {max(ys):.6g}');self.write(7,2,f'X: {min(xs):.6g} .. {max(xs):.6g} | * saved samples; axes auto-scale per channel')
                    if self.other:self.message='Comparison ready: O or G displays both runs on shared axes.'
        if self.tab==6 and self.rows and self.view!='metrics':self.wrap(h-6,2,b.result_help(self.channel),w-5,3,curses.color_pair(2))
        if self.job:self.write(h-4,2,f'SIMULATING {"|/-\\"[int(time.monotonic()*5)%4]} {time.monotonic()-self.started:.1f}s elapsed | X cancel',curses.color_pair(2))
        self.write(h-3,2,self.message,curses.color_pair(2));self.write(h-1,2,'V1 baseline preserved | Every run saves config, effective parameters, CSV, metrics and logs.')
        self.s.refresh()
    def key(self,key):
        if 65<=key<=90 and key!=ord('J'):key=ord(chr(key).lower())
        if key in [curses.KEY_RIGHT,9]:self.tab=(self.tab+1)%8;self.index=self.offset=0
        elif key in [curses.KEY_LEFT,curses.KEY_BTAB]:self.tab=(self.tab-1)%8;self.index=self.offset=0
        elif key in [curses.KEY_DOWN,ord('j')]:self.index+=1;self.offset+=1
        elif key in [curses.KEY_UP,ord('u')]:self.index=max(0,self.index-1);self.offset=max(0,self.offset-1)
        elif key==curses.KEY_NPAGE:self.index+=10;self.offset+=10
        elif key==curses.KEY_PPAGE:self.index=max(0,self.index-10);self.offset=max(0,self.offset-10)
        elif key in [10,13,curses.KEY_ENTER]:
            if self.tab==7 and self.archive:self.load(self.archive[self.index%len(self.archive)])
            else:self.edit()
        elif self.tab==0 and key in [ord('1'),ord('2')]:self.version='V1' if key==ord('1') else 'V2';self.tab=1;self.index=0;self.message=b.VERSIONS[self.version]
        elif key in [ord('r'),ord('R')]:self.start()
        elif key in [ord('a'),ord('A')]:self.scan();self.tab=7;self.index=0
        elif key in [ord('v'),ord('V')]:self.tab=0
        elif key in [ord('x'),ord('X')] and self.job:self.job.terminate();self.message='Cancellation requested. Partial run folder is retained.'
        elif key in [ord('q'),ord('Q')]:
            if self.job:self.message='Simulation active: X cancels it; wait for completion before Q.'
            else:return False
        elif key in [ord('s'),ord('S')]:
            b.OUTPUT.mkdir(exist_ok=True,parents=True);p=b.OUTPUT/('config-'+self.version+'-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json');p.write_text(json.dumps(self.c,indent=2));self.message='Saved '+str(p)
        elif key in [ord('l'),ord('L')]:
            entry=self.prompt('Load config JSON path (original or run config)')
            if entry:
                c=json.loads(Path(entry).expanduser().read_text());b.validate(c);self.configs[c['version']]=c;self.version=c['version'];self.message='Configuration loaded; archived files are unchanged.'
        elif key in [ord('p'),ord('d'),ord('k')]:self.tab=6;self.view={ord('p'):'plot',ord('d'):'data',ord('k'):'metrics'}[key];self.offset=0
        elif key in [ord('['),ord(']')] and self.rows:
            cols=list(self.rows[0]);self.channel=cols[(cols.index(self.channel)+(1 if key==ord(']') else -1))%len(cols)]
        elif key==ord('t') and self.rows:self.tab=6;self.view='plot';self.x='x_m';self.channel='y_m';self.message='XY terminal preview uses independent axis scales. O/G use equal XY aspect.'
        elif key==ord('w'):self.x='time_s'
        elif key in [ord('o'),ord('O')] and self.rows:self.open_file(viewer.export(self.rows,self.source,self.other))
        elif key in [ord('g'),ord('G')] and self.rows:self.open_file(viewer.plot_png(self.rows,self.x,self.channel,self.other))
        elif key==ord('c'):
            if self.tab==7 and self.archive:self.load(self.archive[self.index%len(self.archive)],True)
            else:self.other=[];self.message='Comparison cleared.'
        elif key==ord('J') and hasattr(self,'run_dir'):self.open_file(self.run_dir/'run.log')
        elif key==ord('/'):
            query=self.prompt('Filter archive' if self.tab==7 else 'Find result channel')
            if query:
                if self.tab==7:self.scan();self.archive=[p for p in self.archive if query.lower() in str(p).lower()];self.index=0
                elif self.rows:
                    found=[k for k in self.rows[0] if query.lower() in k.lower()]
                    if found:self.channel=found[0];self.tab=6
        return True
    def run(self):
        curses.curs_set(0);self.s.timeout(100);self.s.keypad(True)
        if curses.has_colors():curses.start_color();curses.use_default_colors();curses.init_pair(1,curses.COLOR_CYAN,-1);curses.init_pair(2,curses.COLOR_YELLOW,-1)
        while True:
            try:self.poll();self.draw();key=self.s.getch()
            except KeyboardInterrupt:
                if self.job:self.job.terminate();self.job.wait();self.log.close()
                break
            if key==-1:continue
            try:
                if self.key(key) is False:break
            except Exception as error:self.message='ERROR: '+str(error)

def main():
    curses.wrapper(lambda screen:Console(screen).run())
if __name__=='__main__':main()
