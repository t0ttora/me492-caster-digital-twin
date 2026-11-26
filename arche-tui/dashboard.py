#!/usr/bin/env python3
"""Mouse-friendly caster dashboard; reuses the existing console's model adapter."""
import curses,json,math,locale,time,textwrap
from pathlib import Path
from app import Console,CHOICES,flat
import backend as b
import viewer

MENU=[('v1','V1  Torque experiments'),('v2','V2  Path tracking'),('run','RUN SIMULATION'),
      ('experiment','Set up a run'),('geometry','Robot dimensions'),('physics','Mass & friction'),
      ('initial','Starting state'),('controller','Controller gains'),('plot','Detailed plot'),
      ('data','Data & metrics'),('archive','Saved runs'),('html','Browser viewer'),
      ('help','How to use'),('quit','Quit')]
GROUPS={'experiment':'experiment','experiment_all':'experiment','geometry':'geometry','physics':'dynamics','initial':'initial','controller':'gains'}
SECTION_KEYS={'1':'v1','2':'v2','3':'experiment','4':'geometry','5':'physics','6':'initial','7':'controller','8':'data','9':'archive','0':'help'}
GUIDE=[
 ('Run an example',[
  'Press 1 for V1 or 2 for V2, then Enter to run. Wait for COMPLETE. Enter opens caster motion; Enter again opens the results. One more Enter returns to setup. To reset edited inputs and run the selected default, press W.']),
 ('Change the inputs',[
  'Each input has a letter. Press A, B or C on setup to edit it. Type the new value and press Enter to apply. Use metres, kilograms, seconds and radians: 50 mm is 0.05 m. Press I for all experiment inputs, or use the numbered left menu for other settings.']),
 ('Apply your settings',[
  'Click Run with these inputs after editing. Changes affect the next run; the displayed saved run keeps its original data. Each model keeps a separate draft. W resets the chosen draft and starts a default run. V1 custom torques need Constant left/right torques in All experiment inputs.']),
 ('Explain the models and plots',[
  'V1 applies wheel torques and calculates motion. V2 follows a path with feedback and attempts to recover the final pose. Purple shows XY motion; pink shows caster/body angles; green shows wheel speeds; orange shows energy. The angle graph uses degrees. Inputs use radians.']),
 ('Inspect the numbers',[
  'Click a graph, then Choose signal for any recorded column. Export PNG saves that plot. Data & metrics has Summary and Table buttons; Signal changes the table column. Browser viewer opens the recorded run in a browser.']),
 ('Read the run status',[
  "RECORDED is an older run. LIVE shows the current run's last 600 samples. COMPLETE loads all saved samples; check metrics to see whether tracking tolerance was reached. STOP ends the run and keeps its partial live trace. FAILED means the run did not finish. These are synthetic results."]),
 ('Open, compare and save',[
  'In Saved runs, click a row and Open selected. To compare, open one run first, then select another and Compare selected. Runs save to arche-tui/outputs. Save settings writes a config file; Load settings asks for its path. Previous results remain saved.']),
 ('Navigate and recover',[
  'Use the numbered menu to open a section. Press the letter beside an input to edit it. N/P changes field pages. Esc goes back; G shows all graphs. Press X to stop a running simulation. Q quits when idle. Tab and arrows remain available, but the printed shortcuts work directly.'])]

def guide_pages(width,capacity):
    pages=[]
    for title,paragraphs in GUIDE:
        lines=[]
        for paragraph in paragraphs:
            if lines:lines.append('')
            lines.extend(textwrap.wrap(paragraph,max(12,width)))
        for offset in range(0,len(lines),max(1,capacity)):
            pages.append((title,lines[offset:offset+max(1,capacity)]))
    return pages
PALETTE={1:(252,235),2:(154,235),3:(252,235),4:(141,235),5:(198,235),6:(154,235),7:(214,235),8:(245,235),9:(16,154),10:(198,235)}
LABELS={'preset':'Experiment','torque_mode':'Wheel torque program','left_torque_Nm':'Left axle torque (N m)',
 'right_torque_Nm':'Right axle torque (N m)','duration_s':'Simulation duration (s)','maneuver':'Path shape',
 'profile':'Timing profile','surface':'Surface preset','design':'Caster geometry preset','segment_distance_m':'Distance per segment (m)',
 'segment_yaw_rad':'Arc angle per segment (rad)','segment_duration_s':'Time per segment (s)','terminal_s':'Final recovery time (s)',
 'mode':'Gain calculation','drive_radius_m':'Drive-wheel radius (m)','drive_track_m':'Drive-wheel spacing (m)',
 'caster_radius_m':'Caster-wheel radius (m)','caster_trail_m':'Caster trail (m)','drive_width_m':'Drive-wheel width (m)',
 'caster_width_m':'Caster-wheel width (m)','body_lumped_mass_kg':'Main assembly mass (kg)',
 'caster_lumped_mass_kg':'Caster assembly mass (kg)','body_yaw_inertia_kgm2':'Body yaw inertia (kg m2)',
 'caster_swivel_inertia_com_kgm2':'Caster swivel inertia (kg m2)','drive_roll_inertia_kgm2':'Drive spin inertia (kg m2)',
 'caster_roll_inertia_kgm2':'Caster spin inertia (kg m2)','gravity_mps2':'Gravity (m/s2)',
 'solver_step_s':'Integration step (s)','command_sample_dt_s':'Command period (s)','result_sample_dt_s':'Recording period (s)',
 'tracking_x':'Forward tracking gain','tracking_y':'Lateral tracking gain','tracking_heading':'Heading tracking gain',
 'wheel_p':'Wheel proportional gain','wheel_i':'Wheel integral gain','terminal_rho':'Final position gain',
 'terminal_alpha':'Final bearing gain','terminal_heading':'Final heading gain'}
PRESETS={'aligned_forward':'Forward / aligned caster','misaligned_forward':'Forward / caster at 45 deg',
 'mirror_misaligned':'Mirror / caster at -45 deg','caster_contact_disabled':'Caster contact ablation','unequal_torque':'Unequal wheel torques',
 'reversal_perturbed':'Forward then reverse','zero_rest':'Rest / zero torque','passive_coast':'Coast / no drive torque',
 'conservative_free':'Conservative / no losses','misaligned_fine':'45 deg / finer step','misaligned_finer':'45 deg / finest step',
 'reversal_no_swivel_resistance':'Reverse / swivel resistance off','straight':'Straight path','arc_turn':'Arc turn',
 'reversal':'Forward and reverse','s_turn':'S-shaped path','baseline':'Constant-speed timing','smooth':'Quintic smooth timing',
 'reference_synthetic':'Reference synthetic surface','high_drag_synthetic':'Higher-drag synthetic surface',
 'nominal':'Nominal caster','auto':'Calculate from design targets','manual':'Use entered gains',
 'constant':'Constant left/right torques','preset':'Original experiment torque program'}

def label(key):
    if key in LABELS:return LABELS[key]
    suffixes={'_kgm2':' (kg m2)','_mps':' (m/s)','_radps':' (rad/s)','_rad':' (rad)','_m':' (m)',
              '_coefficient':' coefficient','_NmspRad':' (N m s/rad)','_Nspm':' (N s/m)','_Nm':' (N m)'}
    name=key
    for suffix,unit in suffixes.items():
        if name.endswith(suffix):return name[:-len(suffix)].replace('_',' ').capitalize()+unit
    return name.replace('_',' ').capitalize()

def parameter_fields(config,group):
    result=[]
    for key,value in config[group].items():
        if key in ['motor_work_J','dissipated_energy_J']:continue
        if isinstance(value,list):
            names={'caster_pivot_m':['Caster pivot X (m)','Caster pivot Y (m)','Caster pivot Z (m)'],
                   'body_com_m':['Body COM X (m)','Body COM Y (m)','Body COM Z (m)'],
                   'chassis_size_m':['Chassis length (m)','Chassis width (m)','Chassis height (m)']}[key]
            result.extend((key,i,names[i],v) for i,v in enumerate(value))
        else:result.append((key,None,label(key),value))
    return result

def raster(series,width,height,equal=False):
    """Connected braille traces on shared axes; each braille dot has square aspect."""
    width=max(1,width);height=max(1,height);pw,ph=width*2,height*4
    points=[(float(x),float(y)) for xs,ys in series for x,y in zip(xs,ys)]
    if not points:return [[' '*width for _ in range(height)] for _ in series],(0,1,0,1)
    xmin=min(x for x,y in points);xmax=max(x for x,y in points);ymin=min(y for x,y in points);ymax=max(y for x,y in points)
    dx=xmax-xmin;dy=ymax-ymin
    if equal:
        scale=max(dx/(pw-1),dy/(ph-1),1e-10)*1.05;dx=scale*(pw-1);dy=scale*(ph-1)
    else:
        dx=(dx or max(abs(xmin)*.1,1e-6))*1.05;dy=(dy or max(abs(ymin)*.1,1e-6))*1.12
    cx=(xmin+xmax)/2;cy=(ymin+ymax)/2;xmin=cx-dx/2;xmax=cx+dx/2;ymin=cy-dy/2;ymax=cy+dy/2
    dots=((1,8),(2,16),(4,32),(64,128));grids=[]
    for xs,ys in series:
        grid=[[0]*width for _ in range(height)];previous=None
        for x,y in zip(xs,ys):
            x=round((x-xmin)/dx*(pw-1));y=round((ymax-y)/dy*(ph-1))
            x=max(0,min(pw-1,x));y=max(0,min(ph-1,y))
            a=previous or (x,y);steps=max(abs(x-a[0]),abs(y-a[1]),1)
            for i in range(steps+1):
                px=round(a[0]+(x-a[0])*i/steps);py=round(a[1]+(y-a[1])*i/steps)
                grid[py//4][px//2]|=dots[py%4][px%2]
            previous=(x,y)
        grids.append([''.join(chr(0x2800+v) if v else ' ' for v in row) for row in grid])
    return grids,(xmin,xmax,ymin,ymax)

class Dashboard(Console):
    def __init__(self,screen,autoload=True):
        super().__init__(screen)
        self.panel='dashboard';self.menu_index=2;self.field_index=0;self.focus='menu';self.hits=[];self.revision=0;self.cache={}
        self.data_mode='summary';self.detail='caster_angle_body_rad';self.data_offset=0;self.live_stamp=0;self.progress=0.;self.cancelled=False;self.result_kind='NO DATA';self.result_version=None
        self.help_page=0
        self.nav_action='run';self.history=[];self.drawing_scope='content'
        self.message='Start with a default example, or edit the inputs below. ? opens the guide.'
        if autoload:self.load_latest()
        self.panel='experiment'
        self.focus='content';self.menu_index=3
        self.message='Press 1 for V1 or 2 for V2, then Enter to run. A-C edits the inputs.'
    def ink(self,pair,bold=False):
        try:a=curses.color_pair(pair)
        except curses.error:a=pair<<8
        return a|(curses.A_BOLD if bold else 0)
    def put(self,y,x,text,pair=1,bold=False,width=None):
        h,w=self.s.getmaxyx()
        if 0<=y<h and 0<=x<w:
            try:self.s.addnstr(y,x,str(text),max(0,min(w-x-1,width if width is not None else w)),self.ink(pair,bold))
            except curses.error:pass
    def lines(self,y,x,text,width,count=3,pair=8):
        for i,line in enumerate(textwrap.wrap(str(text),max(8,width))[:count]):self.put(y+i,x,line,pair,width=width)
    def button(self,y,x,text,action,width=None,active=False,pair=2):
        if self.drawing_scope=='menu':
            active=self.focus=='menu' and action==MENU[self.menu_index][0]
            hotkey=next((k for k,v in SECTION_KEYS.items() if v==action),None)
            if hotkey:text='['+hotkey+'] '+text
            elif action=='run':text='[R] '+text
            elif action=='html':text='[O] '+text
            elif action=='plot':text='[J] Inspect signal'
            elif action=='quit':text='[Q] '+text
        elif self.drawing_scope=='content':active=self.focus=='content' and action==self.nav_action
        width=width or len(text)+2;self.put(y,x,(' '+text+' ').ljust(width),9 if active else pair,active,width)
        self.hits.append((y,x,y+1,x+width,action))
    def invalidate(self):self.revision+=1;self.cache.clear()
    def load(self,path,comparison=False):
        super().load(path,comparison)
        if comparison:self.message='Comparison loaded. Open Detailed plot or Browser viewer to compare.'
        else:
            self.panel='dashboard';self.result_kind='RECORDED';self.result_version=self.metrics.get('version')
            if not self.result_version:
                self.result_version='V2' if 'caster-physics-python-v2' in str(path) or '-V2' in str(path) else ('V1' if 'caster-physics-python/' in str(path) or '-V1' in str(path) else 'ARCHIVE')
            self.message='Recorded results loaded. Click RUN SIMULATION for a new run.'
        self.invalidate()
    def load_latest(self):
        found=sorted(b.OUTPUT.glob('*-'+self.version+'/results.csv'),key=lambda p:p.stat().st_mtime,reverse=True) if b.OUTPUT.exists() else []
        fallback=b.ROOT/('caster-physics-python-v2/evidence/reference_synthetic__straight__baseline__nominal.csv' if self.version=='V2' else 'caster-physics-python/reference/matlab_m2_v1/results/aligned_forward_states.csv')
        if not found and fallback.exists():found=[fallback]
        if found:
            try:self.load(found[0])
            except (ValueError,OSError):self.rows=[];self.source=None;self.message='Click RUN SIMULATION to generate results.'
        else:self.rows=[];self.source=None;self.result_kind='NO DATA'
    def start(self):
        super().start();self.rows=[];self.other=[];self.metrics={};self.source=self.run_dir/'results.csv';self.live_stamp=0;self.progress=0.;self.cancelled=False;self.result_kind='LIVE';self.result_version=self.version;self.panel='dashboard';self.invalidate()
    def poll(self):
        was_running=bool(self.job)
        previous_panel=self.panel
        if self.job:
            path=self.run_dir/'live.json'
            try:
                stamp=path.stat().st_mtime_ns
                if stamp!=self.live_stamp:
                    data=json.loads(path.read_text());self.rows=data['rows'];self.progress=data['progress'];self.live_stamp=stamp;self.invalidate()
            except (OSError,ValueError,KeyError):pass
        super().poll()
        if was_running and not self.job:
            self.panel=previous_panel
            if self.cancelled:self.result_kind='PARTIAL';self.message='Simulation stopped. Partial trace retained; previous runs remain saved.'
            elif self.source and self.source.exists():
                self.result_kind='COMPLETE';self.message='Finished. Enter opens caster motion; Enter again opens results.'
                if self.panel=='dashboard':self.nav_action=('detail','caster_angle_body_rad');self.focus='content'
            else:self.result_kind='FAILED';self.message='Run failed. Open How to use / run log, then review parameters.'
            self.invalidate()
    def fields(self):
        group=GROUPS.get(self.panel)
        if not group:return []
        if group=='gains':
            if self.version=='V1':return []
            if self.c['gains']['mode']=='auto':
                try:self.c['gains'].update(b.active_gains(self.c))
                except (ValueError,ZeroDivisionError,TypeError):pass
        fields=parameter_fields(self.c,group)
        if self.panel=='experiment':
            keys=['preset','duration_s'] if self.version=='V1' else ['maneuver','segment_distance_m','segment_duration_s']
            if self.version=='V1' and self.c['experiment']['torque_mode']=='constant':keys=['left_torque_Nm','right_torque_Nm','duration_s']
            fields=[field for field in fields if field[0] in keys]
        return fields
    def visible_fields(self):
        fields=self.fields()
        count=len(fields) if self.panel=='experiment' else max(1,min(6,self.s.getmaxyx()[0]-18))
        start=0 if self.panel=='experiment' else self.field_index//count*count
        return fields,start,count
    def edit_value(self,field):
        key,component,title,value=field
        if component is None and key in CHOICES:
            options=CHOICES[key];selected=options.index(value) if value in options else 0
            picked=self.dialog(title,[(PRESETS.get(v,v.replace('_',' ')),v) for v in options],selected)
            if picked is None:return
            if key in ['preset','surface','design']:b.select_preset(self.c,key,picked)
            else:self.c[GROUPS[self.panel]][key]=picked
        else:
            result=self.input_number(title,value,b.help_for(key))
            if result is None:return
            if component is None:self.c[GROUPS[self.panel]][key]=result
            else:self.c[GROUPS[self.panel]][key][component]=result
            if self.panel=='controller':self.c['gains']['mode']='manual'
        self.message='Updated '+title+'. Click RUN SIMULATION to apply settings to a new run.'
    def dialog(self,title,options,index=0):
        self.s.timeout(-1)
        try:
            while True:
                self.draw();h,w=self.s.getmaxyx();bw=min(w-6,76);bh=min(h-6,len(options)+5);top=(h-bh)//2;left=(w-bw)//2
                for y in range(top,top+bh):self.put(y,left,' '*bw,1,width=bw)
                self.put(top,left,'─'*bw,8);self.put(top+1,left+2,title,3,True,bw-4);visible=bh-4;start=max(0,min(index-visible//2,len(options)-visible))
                for i,(name,_) in enumerate(options[start:start+visible],start):self.put(top+2+i-start,left+2,f'{i+1}. {name}'.ljust(bw-4),9 if i==index else 1,width=bw-4)
                self.put(top+bh-1,left+2,'1-9: choose / arrows + Enter / Esc: cancel',8,width=bw-4);self.s.refresh();key=self.s.getch()
                if key==27:return None
                if ord('1')<=key<=ord('9') and key-ord('1')<len(options):return options[key-ord('1')][1]
                if key in [10,13,curses.KEY_ENTER]:return options[index][1]
                if key==curses.KEY_UP:index=(index-1)%len(options)
                if key==curses.KEY_DOWN:index=(index+1)%len(options)
                if key==curses.KEY_NPAGE:index=min(len(options)-1,index+visible)
                if key==curses.KEY_PPAGE:index=max(0,index-visible)
                if key==curses.KEY_HOME:index=0
                if key==curses.KEY_END:index=len(options)-1
                if key==curses.KEY_MOUSE:
                    try:_,mx,my,_,state=curses.getmouse()
                    except curses.error:continue
                    pick=start+my-(top+2)
                    if state&(curses.BUTTON1_CLICKED|curses.BUTTON1_RELEASED) and left<=mx<left+bw and 0<=pick<len(options) and top+2<=my<top+bh-2:return options[pick][1]
        finally:self.s.timeout(100)
    def input_number(self,title,value,help_text):
        text=str(value);fresh=True;self.s.timeout(-1);curses.curs_set(1)
        try:
            while True:
                self.draw();h,w=self.s.getmaxyx();bw=min(76,w-6);top=max(2,(h-9)//2);left=(w-bw)//2
                for y in range(top,top+9):self.put(y,left,' '*bw,1,width=bw)
                self.put(top,left,'─'*bw,8);self.put(top+1,left+2,title,3,True,bw-4);self.put(top+3,left+3,('> '+text).ljust(bw-6),9,width=bw-6)
                self.lines(top+5,left+2,help_text,bw-4,2);self.put(top+8,left+2,'Type a number to replace it. Enter = apply. Esc = cancel.',2,width=bw-4)
                self.s.move(top+3,min(left+bw-4,left+5+len(text)));self.s.refresh();key=self.s.get_wch()
                if key=='\x1b':return None
                if key in ['\n','\r']:
                    try:value=float(text)
                    except ValueError:self.message='Please enter a number.';continue
                    if not math.isfinite(value):self.message='Please enter a finite number.';continue
                    return value
                if key in [curses.KEY_BACKSPACE,'\x7f','\b']:text='' if fresh else text[:-1];fresh=False
                elif key=='\x15':text='';fresh=False
                elif isinstance(key,str) and key in '0123456789.eE+-':text=key if fresh else text+key;fresh=False
        finally:curses.curs_set(0);self.s.timeout(100)
    def page_actions(self):
        fields=[('field',i) for i in range(len(self.fields()))]
        if self.panel=='experiment':return ['v1','v2']+fields+['experiment_all','run','help']
        if self.panel in GROUPS:
            count=self.visible_fields()[2]
            return ['dashboard','save','load']+fields+([('fields',-count),('fields',count)] if fields else [])
        if self.panel=='dashboard':return [('detail',k) for k in ['y_m','caster_angle_body_rad','left_wheel_rate_radps','kinetic_energy_J']] if self.rows else []
        if self.panel=='plot':return ['data','dashboard','channels','png']
        if self.panel=='data':return ['experiment','dashboard',('data_mode','summary'),('data_mode','table'),'channels']
        if self.panel=='archive':return ['dashboard','open_run','compare_run']+[('row',i) for i in range(len(self.archive))]
        return ['dashboard','experiment',('help_page',-1),('help_page',1),'log']
    def ensure_navigation(self):
        actions=self.page_actions()
        if self.nav_action not in actions:
            self.nav_action=('run' if self.panel=='experiment' else (('help_page',1) if self.panel=='help' else (('row',self.index) if self.panel=='archive' and self.archive else (('field',min(self.field_index,len(self.fields())-1)) if self.panel in GROUPS and self.fields() else (actions[0] if actions else None)))))
        if isinstance(self.nav_action,tuple):
            if self.nav_action[0]=='field':self.field_index=self.nav_action[1]
            elif self.nav_action[0]=='row':self.index=self.nav_action[1]
    def move_focus(self,delta,tab=False):
        self.ensure_navigation();actions=self.page_actions()
        if tab:
            controls=[('menu',a) for a,_ in MENU]+[('content',a) for a in actions]
            current=('menu',MENU[self.menu_index][0]) if self.focus=='menu' else ('content',self.nav_action)
            index=controls.index(current) if current in controls else 0
            self.focus,selected=controls[(index+delta)%len(controls)]
            if self.focus=='menu':self.menu_index=[a for a,_ in MENU].index(selected)
            else:self.nav_action=selected
        elif self.focus=='menu':self.menu_index=(self.menu_index+delta)%len(MENU)
        elif actions:self.nav_action=actions[(actions.index(self.nav_action)+delta)%len(actions)]
        self.ensure_navigation()
    def back(self):
        if self.history:
            self.panel,self.focus,self.nav_action,self.field_index,self.index,self.menu_index=self.history.pop()
        elif self.panel!='experiment':self.panel='experiment';self.focus='content';self.nav_action=('example',self.version)
        else:self.focus='menu';self.menu_index=3
        return True
    def action(self,action):
        if isinstance(action,tuple) and action[0]=='step':action=action[1]
        if action=='back':return self.back()
        before=(self.panel,self.focus,self.nav_action,self.field_index,self.index,self.menu_index)
        self.nav_action=action
        result=self.perform_action(action)
        if action in ['v1','v2']:self.nav_action='run'
        if isinstance(action,tuple) and action[0]=='field' and self.panel=='experiment':self.nav_action='run'
        if isinstance(action,tuple) and action[0]=='help_page':self.nav_action=('help_page',1)
        if self.panel=='data' and (action=='channels' or isinstance(action,tuple) and action[0]=='data_mode'):self.nav_action='experiment'
        if self.panel=='plot' and action=='channels':self.nav_action='data'
        if self.panel!=before[0]:
            self.history.append(before);self.history=self.history[-40:];self.nav_action=None;self.focus='content'
        self.ensure_navigation()
        return result
    def perform_action(self,action):
        if isinstance(action,tuple):
            kind,value=action
            if kind=='field':self.field_index=value;self.focus='content';self.edit_value(self.fields()[value])
            elif kind=='row':self.index=value;self.focus='content'
            elif kind=='detail':self.detail=value;self.panel='plot';self.focus='content'
            elif kind=='data_mode':self.data_mode=value;self.data_offset=0
            elif kind=='example':
                if self.job:raise ValueError('Stop the current run before starting an example.')
                self.version=value;self.configs[value]=b.defaults(value);self.start()
            elif kind=='help_page':self.help_page=max(0,self.help_page+value)
            elif kind=='fields':self.field_index=max(0,min(self.field_index+value,len(self.fields())-1));self.focus='content'
            return True
        if action in [item[0] for item in MENU]:self.menu_index=[item[0] for item in MENU].index(action)
        if action in ['v1','v2']:
            if self.job:self.message='Stop the current simulation before switching models.';return True
            self.version=action.upper();self.load_latest();self.panel='experiment';self.focus='content';self.field_index=0
            self.message='Selected '+self.version+'. Edit inputs or click Run with these inputs.'
        elif action=='run':
            if self.job:self.cancelled=True;self.job.terminate();self.message='Stopping simulation...'
            else:self.start()
        elif action=='dashboard':self.panel='dashboard';self.focus='content'
        elif action in GROUPS:self.panel=action;self.field_index=0;self.focus='content'
        elif action=='archive':self.scan();self.panel='archive';self.index=0;self.focus='content'
        elif action in ['plot','data','help']:self.panel=action;self.data_offset=0;self.focus='content'
        elif action=='channels':
            if not self.rows:raise ValueError('Run a simulation or open a saved run first.')
            keys=list(self.rows[0]);picked=self.dialog('Choose the signal to inspect',[(label(k),k) for k in keys],keys.index(self.detail) if self.detail in keys else 0)
            if picked:self.detail=picked
        elif action in ['html','png']:
            if self.job or not self.rows:raise ValueError('Finish the simulation or open a saved run first.')
            path=viewer.export(self.rows,self.source,self.other) if action=='html' else viewer.plot_png(self.rows,'time_s',self.detail,self.other)
            self.open_file(path)
        elif action in ['open_run','compare_run']:
            if self.archive:
                if action=='compare_run' and not self.rows:raise ValueError('Open a primary run first, then choose a comparison.')
                self.load(self.archive[self.index%len(self.archive)],comparison=action=='compare_run')
                if action=='compare_run':self.panel='plot';self.focus='content'
        elif action=='save':Console.key(self,ord('s'))
        elif action=='load':Console.key(self,ord('l'));self.load_latest()
        elif action=='log':
            if hasattr(self,'run_dir'):self.open_file(self.run_dir/'run.log')
            else:self.message='No run has started in this session. Open Set up a run to begin.'
        elif action=='quit':
            if self.job:self.message='Simulation is running. Click STOP SIMULATION, then Quit.'
            else:return False
        return True
    def chart(self,y,x,height,width,title,series,legends,colors,equal=False,detail=None):
        if height<3 or width<16:return
        selected=detail is not None and self.focus=='content' and self.nav_action==('detail',detail)
        self.put(y,x,'─'*width,8,width=width);self.put(y,x+2,(' > '+title+' [Enter] ') if selected else ' '+title+' ',9 if selected else colors[0],True,width-4)
        plot_h=max(1,height-4);plot_w=max(2,width-12);cachekey=(self.revision,title,plot_w,plot_h,detail)
        if cachekey not in self.cache:self.cache[cachekey]=raster(series,plot_w,plot_h,equal)
        grids,limits=self.cache[cachekey];xmin,xmax,ymin,ymax=limits
        self.put(y+1,x,f'{ymax:9.3g}',8,width=10)
        for grid,color in reversed(list(zip(grids,colors))):
            for row,line in enumerate(grid):
                for col,char in enumerate(line):
                    if char!=' ':self.put(y+1+row,x+11+col,char,color)
        if plot_h>1:self.put(y+plot_h,x,f'{ymin:9.3g}',8,width=10)
        if height>=5:
            self.put(y+height-2,x+11,f'{xmin:.3g}',8);self.put(y+height-2,x+width-13,f'{xmax:.3g}',8,width=12)
        legend='  /  '.join(legends)
        self.put(y+height-1,x+2,legend,8,width=width-4)
        if detail:self.hits.append((y,x,y+height,x+width,('detail',detail)))
    def sidebar(self,h,w,left):
        for y in range(2,h-2):self.put(y,left,'│',8)
        self.put(2,2,'MODEL',3,True)
        self.button(3,1,'V1 Torques'+(' *' if self.version=='V1' else ''),'v1',left-2,pair=2)
        self.button(4,1,'V2 Tracking'+(' *' if self.version=='V2' else ''),'v2',left-2,pair=2)
        self.lines(6,2,'Wheel torque → motion' if self.version=='V1' else 'Reference path → feedback → wheel torques',left-4,2)
        self.button(9,1,'STOP SIMULATION' if self.job else 'RUN SIMULATION','run',left-2,active=self.menu_index==2,pair=10 if self.job else 2)
        for i,(action,title) in enumerate(MENU[3:-1],3):
            y=11+i-3
            if y>=h-3:break
            self.button(y,1,title,action,left-2,active=self.menu_index==i)
        self.button(h-3,1,'Quit','quit',left-2,active=self.menu_index==len(MENU)-1)
        if h>=35:
            self.put(25,2,'LATEST VALUES',3,True)
            if self.rows:
                for i,(name,key,unit) in enumerate([('Time','time_s','s'),('Axle X','x_m','m'),('Axle Y','y_m','m'),('Body yaw','yaw_rad','rad')]):self.put(27+i,2,f'{name:<9} {self.rows[-1].get(key,0):.4g} {unit}',8,width=left-3)
    def title(self,title,x,y,width):
        self.button(y,x,'[G] All graphs','dashboard')
        self.put(y+2,x,title,3,True,width)
    def draw(self):
        self.s.erase();self.hits=[];h,w=self.s.getmaxyx();left=max(24,min(30,w//4));x=left+3;width=w-x-2
        self.ensure_navigation()
        self.put(0,2,'ARCHE  /  CASTER DYNAMICS LAB',3,True);self.put(0,max(40,w-25),'SYNTHETIC · SI UNITS',8)
        self.drawing_scope='top'
        stage='experiment' if self.panel in GROUPS else ('dashboard' if self.panel=='plot' else ('data' if self.panel=='archive' else self.panel))
        for col,text,target in [(1,'S Setup','experiment'),(14,'R Run','run'),(25,'G Graphs','dashboard'),(40,'M Results','data')]:
            self.button(1,col,text,('step',target),pair=4 if target==stage else 8)
        for col in [12,23,38]:self.put(1,col,'>',8)
        self.drawing_scope='menu'
        self.sidebar(h,w,left)
        self.drawing_scope='content'
        if w<76 or h<23:self.lines(6,x,'Enlarge Terminal to at least 76 x 23. Fullscreen gives larger charts.',max(10,width),4,2)
        else:
            status=self.result_kind+' '+str(self.result_version or self.version)
            if self.job:status=f'LIVE {self.version}   {self.progress:.0%}   {time.monotonic()-self.started:.1f}s elapsed   (last 600 samples)'
            elif self.rows:status+=f'   {len(self.rows)} samples   t={self.rows[-1]["time_s"]:.3g}s'
            self.put(2,x,status,2 if self.job else 8,width=width)
            if self.source:self.put(3,x,'Run: '+self.source.parent.name+' / '+self.source.name,8,width=width)
            if self.panel=='dashboard':self.draw_dashboard(x,4,width,h-8)
            elif self.panel=='experiment':self.draw_setup(x,4,width,h-9)
            elif self.panel in GROUPS:self.draw_parameters(x,4,width,h-9)
            elif self.panel=='archive':self.draw_archive(x,4,width,h-9)
            elif self.panel=='plot':self.draw_detail(x,4,width,h-9)
            elif self.panel=='data':self.draw_data(x,4,width,h-9)
            else:self.draw_help(x,4,width,h-9)
        hint={'experiment':f'Enter: run | A-{chr(64+len(self.fields()))}: edit | I: all inputs | W: default',
              'dashboard':'Enter: caster motion | A: angles B: XY C: wheels D: energy',
              'plot':'Enter: results | C: choose signal | E: export PNG',
              'data':'Enter: setup | T: table | Y: summary | C: signal | Arrows: scroll',
              'archive':'Arrows: choose | Enter: open | C: compare',
              'help':'Enter / N: next page | P: previous page'}.get(self.panel,'A-F: edit field | N/P: field pages | R: run | V: save L: load')
        if self.job:hint='Running... X: stop | 8: results | 0: help'
        self.put(h-2,2,self.message,10 if self.message.startswith(('Check settings:','Run failed')) else 2,width=w-4);self.put(h-1,2,hint+' | Esc: back',8,width=w-4)
        self.s.refresh()
    def draw_setup(self,x,y,width,height):
        compact=height<22
        self.put(y,x,'Set up a run',3,True,width)
        self.button(y+1,x,'[1] V1: torques','v1')
        self.button(y+1,x+21,'[2] V2: tracking','v2')
        self.lines(y+2,x,'Pick a model, then press Enter. A-C edits inputs.',width,2)
        base=y+4
        self.put(base,x,'Your inputs: '+self.version,3,True,width)
        self.field_index=max(0,min(self.field_index,len(self.fields())-1))
        for i,(key,comp,title,value) in enumerate(self.fields()):
            display=PRESETS.get(value,value) if isinstance(value,str) else f'{value:.7g}'
            self.button(base+1+i,x,f'[{chr(65+i)}] {title}: {display}',('field',i),width,active=self.focus=='content' and i==self.field_index,pair=1)
        row=base+1+len(self.fields())
        self.button(row+1,x,'[I] All experiment inputs','experiment_all')
        self.button(row+2,x,'[X] Stop simulation' if self.job else '[Enter] Run with these inputs','run',active=not self.job)
        self.button(row+3,x,'[0] Read the guide','help')
        if not compact:
            explanation=('V1 applies wheel torques and calculates caster motion.' if self.version=='V1' else 'V2 follows a path and attempts to recover the final pose.')
            self.lines(row+5,x,explanation,width,2,1)
            self.lines(row+8,x,'After the run, click a graph to inspect it. Data & metrics shows the numbers. F3 returns to the four graphs.',width,3)
    def draw_dashboard(self,x,y,width,height):
        if not self.rows:
            self.put(y+3,x+2,'READY TO SIMULATE',3,True);self.lines(y+6,x+2,'Choose V1 or V2 on the left, then click RUN SIMULATION. The four plots will update while the model runs.',width-4,5,2);return
        rows=self.rows;times=[r['time_s'] for r in rows]
        def col(key,deg=False):return [math.degrees(r.get(key,0)) if deg else r.get(key,0) for r in rows]
        blocks=[('[B] TRAJECTORY · X / Y [m]',[(col('x_m'),col('y_m'))],[f'X {rows[-1]["x_m"]:.4g} m · Y {rows[-1]["y_m"]:.4g} m · equal axes'],[4],True,'y_m'),
          ('[A] ANGLES [deg]',[(times,col('caster_angle_body_rad',True)),(times,col('yaw_rad',True))],[f'Caster {col("caster_angle_body_rad",True)[-1]:.3g} deg',f'Yaw {col("yaw_rad",True)[-1]:.3g} deg'],[5,8],False,'caster_angle_body_rad'),
          ('[C] WHEEL SPEED [rad/s]',[(times,col('left_wheel_rate_radps')),(times,col('right_wheel_rate_radps'))],[f'Left {col("left_wheel_rate_radps")[-1]:.3g}',f'Right {col("right_wheel_rate_radps")[-1]:.3g} rad/s'],[6,8],False,'left_wheel_rate_radps'),
          ('[D] ENERGY [J]',[(times,col('kinetic_energy_J')),(times,col('dissipated_energy_J'))],[f'Kinetic {col("kinetic_energy_J")[-1]:.3g} J',f'Loss {col("dissipated_energy_J")[-1]:.3g} J'],[7,8],False,'kinetic_energy_J')]
        hh=max(3,height//4)
        for i,(title,series,legends,colors,equal,detail) in enumerate(blocks):self.chart(y+i*hh,x,hh,width,title,series,legends,colors,equal,detail)
    def draw_parameters(self,x,y,width,height):
        name='All experiment inputs' if self.panel=='experiment_all' else next(title for action,title in MENU if action==self.panel);self.title(name,x,y,width)
        self.button(y,x+16,'[V] Save','save');self.button(y,x+29,'[L] Load','load')
        fields=self.fields()
        if not fields:self.lines(y+5,x,'V1 uses wheel torques, not a trajectory controller. Choose V2 to edit feedback gains.',width,4);return
        self.field_index=max(0,min(self.field_index,len(fields)-1));fields,start,visible=self.visible_fields()
        for i,(key,comp,title,value) in enumerate(fields[start:start+visible],start):
            display=PRESETS.get(value,value) if isinstance(value,str) else f'{value:.7g}'
            text=('['+chr(65+i-start)+'] '+title)[:max(12,width-23)].ljust(max(12,width-23))+'  '+str(display)
            self.button(y+4+i-start,x,text,('field',i),width,active=self.focus=='content' and i==self.field_index,pair=1)
        key,_,title,_=fields[self.field_index];self.lines(y+height-3,x,b.help_for(key),width,3,8)
        self.button(y+height-5,x,'[P] Previous',('fields',-visible))
        self.button(y+height-5,x+16,'[N] Next',('fields',visible))
        self.put(y+height-4,x,f'Fields {start+1}-{min(start+visible,len(fields))} of {len(fields)}. Press a letter to edit.',8,width=width)
    def draw_archive(self,x,y,width,height):
        self.title('Saved simulation results',x,y,width)
        by=y if width>=65 else y+1
        bx=x+17 if width>=65 else x
        self.button(by,bx,'Open selected','open_run');self.button(by,bx+18,'Compare selected','compare_run')
        if not self.archive:self.lines(y+5,x,'No saved results found. Run a simulation first.',width,3);return
        self.index=max(0,min(self.index,len(self.archive)-1));visible=max(1,height-7);start=max(0,min(self.index-visible//2,len(self.archive)-visible))
        for i,path in enumerate(self.archive[start:start+visible],start):
            version='V2' if 'v2' in str(path).lower() else 'V1';name=path.stem if path.stem!='results' else path.parent.name
            self.button(y+4+i-start,x,version+' | '+name.replace('__',' / ').replace('_',' '),('row',i),width,active=self.focus=='content' and i==self.index,pair=1)
        self.lines(y+height-2,x,str(self.archive[self.index].relative_to(b.ROOT)),width,2)
    def draw_detail(self,x,y,width,height):
        self.title('Inspect any recorded signal',x,y,width);self.button(y,x+17,'Choose signal','channels');self.button(y,x+34,'Export PNG','png')
        if not self.rows:self.lines(y+5,x,'Run a simulation or open a saved run first.',width,3);return
        if self.detail not in self.rows[0]:self.detail=next(iter(self.rows[0]))
        series=[([r['time_s'] for r in self.rows],[r[self.detail] for r in self.rows])];legends=['Selected run'];colors=[4]
        if self.other and self.detail in self.other[0]:series.append(([r['time_s'] for r in self.other],[r[self.detail] for r in self.other]));legends.append('Comparison');colors.append(7)
        self.chart(y+4,x,max(4,height-8),width,label(self.detail),series,legends,colors)
        self.lines(y+height-3,x,b.result_help(self.detail),width,2)
        self.button(y+height-1,x,'[Enter] Next: results','data')
    def draw_data(self,x,y,width,height):
        self.title('Data & metrics',x,y,width);self.button(y,x+17,'Summary',('data_mode','summary'),active=self.data_mode=='summary');self.button(y,x+29,'Table',('data_mode','table'),active=self.data_mode=='table');self.button(y,x+39,'Signal','channels')
        if not self.rows:self.lines(y+5,x,'Run a simulation or open a saved run first.',width,3);return
        if self.data_mode=='summary':
            items=flat(self.metrics);self.data_offset=max(0,min(self.data_offset,max(0,len(items)-1)))
            for i,text in enumerate(items[self.data_offset:self.data_offset+max(1,height-6)]):self.put(y+4+i,x,text,1,width=width)
        else:
            self.data_offset=max(0,min(self.data_offset,max(0,len(self.rows)-height+7)));key=self.detail if self.detail in self.rows[0] else 'x_m'
            self.put(y+4,x,f'{"Time (s)":>12}  {label(key)}',3,True,width)
            for i,row in enumerate(self.rows[self.data_offset:self.data_offset+max(1,height-7)]):self.put(y+5+i,x,f'{row["time_s"]:12.5g}  {row[key]:18.9g}',1,width=width)
        self.button(y+height-1,x,'[Enter] Next: setup / another model','experiment')
    def draw_help(self,x,y,width,height):
        self.title('How to use',x,y,width)
        self.button(y,x+17,'Set up a run','experiment')
        pages=guide_pages(width,max(1,height-9))
        self.help_page=max(0,min(self.help_page,len(pages)-1))
        title,lines=pages[self.help_page]
        self.put(y+2,x,f'{self.help_page+1}/{len(pages)}  {title}',3,True,width)
        for i,line in enumerate(lines):self.put(y+4+i,x,line,1,width=width)
        self.button(y+height-4,x,'Previous',('help_page',-1))
        self.button(y+height-4,x+13,'Next',('help_page',1))
        self.put(y+height-3,x,'P / N: change page | Esc: back | F3: graphs',8,width=width)
        self.button(y+height-1,x,'Open last run log','log')
    def scroll(self,delta):
        if self.panel=='data':self.data_offset=max(0,self.data_offset+delta)
        elif self.panel=='help':self.action(('help_page',1 if delta>0 else -1))
        else:self.move_focus(delta)
    def key(self,key):
        if key==curses.KEY_MOUSE:
            try:_,mx,my,_,state=curses.getmouse()
            except curses.error:return True
            if state&curses.BUTTON4_PRESSED:self.scroll(-1);return True
            if state&getattr(curses,'BUTTON5_PRESSED',0):self.scroll(1);return True
            if state&(curses.BUTTON1_CLICKED|curses.BUTTON1_RELEASED):
                for y0,x0,y1,x1,action in reversed(self.hits):
                    if y0<=my<y1 and x0<=mx<x1:
                        left=max(24,min(30,self.s.getmaxyx()[1]//4))
                        self.focus='menu' if x0<=left and y0!=1 else 'content'
                        result=self.action(action)
                        if x0<=left and y0!=1 and action in [a for a,_ in MENU] and self.panel not in GROUPS:
                            self.focus='content'
                        return result
            return True
        if 0<=key<256:
            letter=chr(key).lower()
            if letter in SECTION_KEYS:return self.action(SECTION_KEYS[letter])
            if self.panel in GROUPS and letter in 'abcdef':
                fields,start,count=self.visible_fields();index=start+ord(letter)-ord('a')
                if index<min(start+count,len(fields)):return self.action(('field',index))
                return True
            if self.panel in GROUPS and letter in 'np' and self.panel!='experiment':
                return self.action(('fields',self.visible_fields()[2]*(1 if letter=='n' else -1)))
            if letter=='w':return self.action(('example',self.version))
            if letter=='i':return self.action('experiment_all')
            if letter=='x':
                if self.job:return self.action('run')
                self.message='No simulation is running.';return True
            if self.panel=='dashboard' and letter in 'abcd':
                return self.action(('detail',dict(a='caster_angle_body_rad',b='y_m',c='left_wheel_rate_radps',d='kinetic_energy_J')[letter]))
            if self.panel=='plot' and letter in 'ce':return self.action('channels' if letter=='c' else 'png')
            if self.panel=='data' and letter in 'tyc':return self.action('channels' if letter=='c' else ('data_mode','table' if letter=='t' else 'summary'))
            if self.panel=='archive' and letter in 'oc':return self.action('open_run' if letter=='o' else 'compare_run')
            if letter=='o':return self.action('html')
            if letter in 'vl' and self.panel in GROUPS:return self.action('save' if letter=='v' else 'load')
        shortcuts={curses.KEY_F1:'help',curses.KEY_F2:'experiment',curses.KEY_F3:'dashboard',curses.KEY_F4:'data',curses.KEY_F5:'run',ord('?'):'help',ord('r'):'run',ord('R'):'run',ord('q'):'quit',ord('Q'):'quit',ord('1'):'v1',ord('2'):'v2'}
        for letter,action in [('s','experiment'),('g','dashboard'),('m','data'),('h','help'),('j','plot')]:
            shortcuts[ord(letter)]=action;shortcuts[ord(letter.upper())]=action
        if key in shortcuts:return self.action(shortcuts[key])
        if key in [27,curses.KEY_BACKSPACE,127,8]:return self.back()
        if key==curses.KEY_F6:
            self.focus='content' if self.focus=='menu' else 'menu';self.ensure_navigation();return True
        if key==9:self.move_focus(1,tab=True);return True
        if key==curses.KEY_BTAB:self.move_focus(-1,tab=True);return True
        if key==curses.KEY_LEFT:self.focus='menu';return True
        if key==curses.KEY_RIGHT:self.focus='content';self.ensure_navigation();return True
        if self.panel=='help' and key in [ord('n'),ord('N'),ord('p'),ord('P')]:return self.action(('help_page',1 if key in [ord('n'),ord('N')] else -1))
        if key in [curses.KEY_NPAGE,curses.KEY_PPAGE]:
            self.scroll((1 if key==curses.KEY_NPAGE else -1)*max(1,self.s.getmaxyx()[0]-12));return True
        if key in [curses.KEY_UP,curses.KEY_DOWN]:
            delta=1 if key==curses.KEY_DOWN else -1
            if self.focus=='content' and self.panel=='data':self.scroll(delta)
            else:self.move_focus(delta)
            return True
        if key in [curses.KEY_HOME,curses.KEY_END]:
            if self.focus=='menu':self.menu_index=0 if key==curses.KEY_HOME else len(MENU)-1
            else:
                actions=self.page_actions()
                if actions:self.nav_action=actions[0] if key==curses.KEY_HOME else actions[-1];self.ensure_navigation()
            return True
        if key in [10,13,curses.KEY_ENTER]:
            if self.job:self.message='Simulation is running. Wait for COMPLETE, or press X to stop.';return True
            self.ensure_navigation()
            action=MENU[self.menu_index][0] if self.focus=='menu' else self.nav_action
            if action is not None:
                result=self.action(action)
                if isinstance(action,tuple) and action[0]=='row':return self.action('open_run')
                return result
        return True
    def run(self):
        curses.curs_set(0);self.s.timeout(100);self.s.keypad(True)
        if curses.has_colors():
            curses.start_color();curses.use_default_colors()
            for i,(fg,bg) in PALETTE.items():
                if curses.COLORS<256:fg={1:7,2:2,3:7,4:5,5:1,6:2,7:3,8:7,9:0,10:1}[i];bg=2 if i==9 else -1
                curses.init_pair(i,fg,bg)
            self.s.bkgd(' ',self.ink(1))
        curses.mousemask(curses.ALL_MOUSE_EVENTS);curses.mouseinterval(120)
        try:
            while True:
                self.poll();self.draw();key=self.s.getch()
                if key==-1:continue
                try:
                    if self.key(key) is False:break
                except Exception as error:self.message='Check settings: '+str(error)
        except KeyboardInterrupt:
            if self.job:self.job.terminate();self.job.wait();self.log.close()

def main():
    locale.setlocale(locale.LC_ALL,'');curses.wrapper(lambda s:Dashboard(s).run())
if __name__=='__main__':main()
