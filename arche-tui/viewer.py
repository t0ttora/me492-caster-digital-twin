"""Offline plots and a selectable-channel interactive viewer of recorded results."""
from pathlib import Path
import json, html
from datetime import datetime
import backend as b

def export(rows,source,other=None):
    out=b.OUTPUT/'exports';out.mkdir(parents=True,exist_ok=True)
    stem=datetime.now().strftime('%Y%m%d-%H%M%S-%f');dest=out/(stem+'.html')
    payload=json.dumps({'rows':rows,'other':other or []},allow_nan=False).replace('<',r'\u003c')
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Arche recorded results</title>
<style>body{font:15px system-ui;margin:28px;max-width:1250px;color:#19252e}h1{font-size:24px}label{margin-right:18px}select,button,input{padding:7px;margin:8px 4px}canvas{width:100%;height:440px;border:1px solid #ccd3d8}p{line-height:1.5}small{color:#56626b}table{border-collapse:collapse;font:12px monospace}td,th{padding:5px 10px;border-bottom:1px solid #ddd;text-align:right}.data{overflow:auto;max-height:350px}summary{cursor:pointer;margin:20px 0}</style>
<h1>Arche | Recorded simulation explorer</h1><p>SOURCE</p><p><small>Synthetic model output. Charts and playback use saved samples; this viewer performs no physics calculation. Blue: selected run. Orange: comparison. Dashed: reference when available.</small></p>
<label>X channel <select id="x"></select></label><label>Y channel <select id="y"></select></label>
<button id="play">Play / pause</button><button id="full">Full trace</button><input id="frame" type="range" min="1" value="1"><span id="time"></span>
<canvas id="plot"></canvas><p id="stats"></p><details><summary>Recorded numeric data (first 500 rows; complete data in source CSV)</summary><div class="data"><table id="data"></table></div></details>
<script>const datasets=PAYLOAD,rows=datasets.rows,other=datasets.other,keys=Object.keys(rows[0]);
const $=id=>document.getElementById(id),x=$('x'),y=$('y'),slider=$('frame');
for(const key of keys){for(const e of [x,y]){let o=document.createElement('option');o.value=o.textContent=key;e.append(o)}}
x.value='time_s';y.value=keys.includes('caster_angle_body_rad')?'caster_angle_body_rad':keys[1];slider.max=slider.value=rows.length;let playing=false;
function draw(){let cvs=$('plot'),W=cvs.width=Math.round(cvs.clientWidth*devicePixelRatio),H=cvs.height=Math.round(440*devicePixelRatio),ctx=cvs.getContext('2d');ctx.scale(devicePixelRatio,devicePixelRatio);W/=devicePixelRatio;H/=devicePixelRatio;
let n=+slider.value,points=rows.slice(0,n),sets=[{points,color:'#176a91'}];if(other.length&&x.value in other[0]&&y.value in other[0])sets.push({points:other,color:'#c15d26'});
if(x.value==='x_m'&&y.value==='y_m'&&'ref_x_m' in rows[0])sets.push({points:rows.map(r=>({...r,x_m:r.ref_x_m,y_m:r.ref_y_m})),color:'#69737b',dash:true});
let all=sets.flatMap(s=>s.points),minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;for(let r of all){minX=Math.min(minX,r[x.value]);maxX=Math.max(maxX,r[x.value]);minY=Math.min(minY,r[y.value]);maxY=Math.max(maxY,r[y.value])}
let dx=maxX-minX||1,dy=maxY-minY||1;if(x.value==='x_m'&&y.value==='y_m'){let scale=Math.max(dx/(W-120),dy/(H-90)),cx=(minX+maxX)/2,cy=(minY+maxY)/2;dx=scale*(W-120);dy=scale*(H-90);minX=cx-dx/2;maxX=cx+dx/2;minY=cy-dy/2;maxY=cy+dy/2}ctx.clearRect(0,0,W,H);ctx.font='12px system-ui';ctx.fillStyle='#19252e';ctx.fillText(y.value,12,20);ctx.fillText(x.value,W/2,H-8);
for(let j=0;j<=5;j++){let yy=35+(H-90)*j/5;ctx.strokeStyle='#e6ebee';ctx.beginPath();ctx.moveTo(95,yy);ctx.lineTo(W-25,yy);ctx.stroke();ctx.fillText((maxY-dy*j/5).toPrecision(5),8,yy+4);ctx.fillText((minX+dx*j/5).toPrecision(5),90+(W-125)*j/5,H-35)}
for(let s of sets){ctx.strokeStyle=s.color;ctx.lineWidth=1.7;ctx.setLineDash(s.dash?[6,4]:[]);ctx.beginPath();s.points.forEach((r,i)=>{let xx=95+(r[x.value]-minX)/dx*(W-120),yy=35+(maxY-r[y.value])/dy*(H-90);i?ctx.lineTo(xx,yy):ctx.moveTo(xx,yy)});ctx.stroke()}ctx.setLineDash([]);
$('time').textContent='t = '+points.at(-1).time_s.toFixed(3)+' s';$('stats').textContent='Visible samples: '+n+' / '+rows.length+' | '+y.value+' range: '+minY.toPrecision(6)+' to '+maxY.toPrecision(6)}
x.onchange=y.onchange=slider.oninput=draw;$('full').onclick=()=>{slider.value=rows.length;playing=false;draw()};$('play').onclick=()=>{if(+slider.value>=rows.length)slider.value=1;playing=!playing};setInterval(()=>{if(playing){slider.value=Math.min(rows.length,+slider.value+Math.max(1,Math.floor(rows.length/150)));draw();if(+slider.value===rows.length)playing=false}},50);window.onresize=draw;
let head=$('data').createTHead().insertRow();keys.forEach(k=>{let t=document.createElement('th');t.textContent=k;head.append(t)});let body=$('data').createTBody();rows.slice(0,500).forEach(r=>{let line=body.insertRow();keys.forEach(k=>line.insertCell().textContent=r[k].toPrecision(7))});draw();</script></html>'''
    page=page.replace('SOURCE',html.escape(str(source))).replace('PAYLOAD',payload)
    with dest.open('x') as f:f.write(page)
    return dest

def plot_png(rows,x,y,other=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=b.OUTPUT/'exports';out.mkdir(parents=True,exist_ok=True)
    path=out/(datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.png')
    fig,ax=plt.subplots(figsize=(10,6));ax.plot([r[x] for r in rows],[r[y] for r in rows],label='Selected run')
    if other and x in other[0] and y in other[0]:ax.plot([r[x] for r in other],[r[y] for r in other],label='Comparison',alpha=.8)
    if x=='x_m' and y=='y_m':
        ax.set_aspect('equal',adjustable='datalim')
        if 'ref_x_m' in rows[0]:ax.plot([r['ref_x_m'] for r in rows],[r['ref_y_m'] for r in rows],'--',label='Reference')
    ax.set(xlabel=x,ylabel=y,title='Arche | recorded synthetic simulation');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(path,dpi=180);plt.close(fig);return path
