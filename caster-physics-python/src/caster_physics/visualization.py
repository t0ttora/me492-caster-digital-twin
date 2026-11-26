"""OKUMA 13 — Kayıtları çiz: bu dosya yeni hareket veya fizik üretmez.

A tahrik aksının orta noktasıdır; B ise gövdenin kütle merkezidir (COM).
Animasyon yalnız kayıtlı çözüm örneklerini oynatır, ara fizik hesabı yapmaz.
"""
from dataclasses import asdict
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from .result_files import atomic_output, prepare_output_directory, validate_output_file


_TITLES = {
    'aligned_forward': 'Caster başlangıçta 0°',
    'misaligned_forward': 'Caster başlangıçta 45°',
    'passive_coast': 'Motorsuz serbest yavaşlama',
}
_COLORS = ['#176a76', '#b24f31', '#765899']


def _selected(results, limit):
    if not results:
        raise ValueError('Görselleştirmek için en az bir deney gerekli.')
    preferred = [name for name in _TITLES if name in results]
    remaining = [name for name in results if name not in preferred]
    return [(name, results[name]) for name in (preferred+remaining)[:limit]]


def create_plots(results, output_dir):
    """Üç temel deneyi iki PNG'de karşılaştır; sayılar SI kayıtlarından gelir."""
    selected = _selected(results, 3)
    output_dir = prepare_output_directory(output_dir)
    files = [output_dir/'trajectory_comparison.png', output_dir/'angles_and_energy.png']
    for file in files:
        validate_output_file(file)
    with plt.rc_context({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False}):
        figure, axis = plt.subplots(figsize=(9, 6), constrained_layout=True)
        for color, (name, result) in zip(_COLORS, selected):
            x, y = result.column('x_m'), result.column('y_m')
            axis.plot(x, y, color=color, label=_TITLES.get(name, name), linewidth=2)
            axis.scatter(x[0], y[0], color=color, marker='o', s=20)
            axis.scatter(x[-1], y[-1], color=color, marker='x', s=40)
        axis.set(xlabel='Dünya X [m]', ylabel='Dünya Y [m]',
                 title='A: tahrik aksı orta noktasının yörüngesi\nDaire: başlangıç · çarpı: bitiş')
        axis.axis('equal')
        axis.grid(alpha=.2)
        axis.legend()
        figure.suptitle('Sentetik parametreli model — gerçek robot ölçümü değildir', fontsize=11)
        with atomic_output(files[0]) as temporary:
            figure.savefig(temporary, dpi=160)
        plt.close(figure)

        figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        series = [('yaw_rad', 'Gövde yönelimi theta [°]', True),
                  ('caster_angle_body_rad', 'Gövdeye göre caster açısı phi [°]', True),
                  ('kinetic_energy_J', 'Kinetik enerji [J]', False),
                  ('energy_balance_error_J', 'Enerji bilançosu sayısal hatası [J]', False)]
        for axis, (column, ylabel, degrees) in zip(axes.flat, series):
            for color, (name, result) in zip(_COLORS, selected):
                values = result.column(column)
                if degrees:
                    values = np.rad2deg(values)
                axis.plot(result.column('time_s'), values, color=color,
                          label=_TITLES.get(name, name), linewidth=1.8)
            axis.set(xlabel='Zaman [s]', ylabel=ylabel)
            axis.grid(alpha=.2)
        axes[0, 0].legend(fontsize=9)
        figure.suptitle('Sentetik model · Açılar ve enerji\nBilanço: E(t) − E(0) + kayıp − motor işi', fontsize=12)
        with atomic_output(files[1]) as temporary:
            figure.savefig(temporary, dpi=160)
        plt.close(figure)
    return files


def _animation_case(name, result):
    """A'dan gövde noktalarını döndür; caster tekeri pivottan trail kadar geridedir."""
    geometry = result.parameters.geometry
    samples = []
    for row in result.records:
        theta, phi = row['yaw_rad'], row['caster_angle_body_rad']
        rotation = np.array([[np.cos(theta), -np.sin(theta)],
                             [np.sin(theta), np.cos(theta)]])
        axle = np.array([row['x_m'], row['y_m']])
        pivot = axle+rotation @ np.array(geometry.caster_pivot_m[:2])
        caster = pivot-geometry.caster_trail_m*np.array([np.cos(theta+phi), np.sin(theta+phi)])
        samples.append({'time': row['time_s'], 'theta': theta, 'phi': phi,
                        'axle': axle.tolist(),
                        'com': [row['body_com_x_world_m'], row['body_com_y_world_m']],
                        'pivot': pivot.tolist(), 'caster': caster.tolist()})
    if not samples:
        raise ValueError('Her deney en az bir kayıt içermeli.')
    return {'name': name, 'title': _TITLES.get(name, name),
            'geometry': asdict(geometry), 'samples': samples}


def create_animation(results, output_path):
    """Çevrimdışı açılabilen, duraklatılmış başlayan üstten görünüş HTML'i yaz."""
    cases = [_animation_case(name, result) for name, result in _selected(results, 2)]
    # allow_nan=False bozuk sayıları reddeder; '<' kaçışı script kapanmasını önler.
    data = json.dumps({'cases': cases}, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    output_path = Path(output_path)
    with atomic_output(output_path) as temporary:
        temporary.write_text(_HTML.replace('__SIMULATION_DATA__', data), encoding='utf-8')
    return output_path


_HTML = '''<!doctype html>
<html lang="tr"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Caster fiziği — kayıtlı hareket</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f3f2ed;color:#183138;font:16px system-ui,sans-serif}
main{max-width:1240px;margin:auto;padding:24px}h1{font-size:26px;margin-bottom:8px}p{line-height:1.5}
.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:20px 0}button{font:inherit;padding:9px 16px;background:#176a76;color:white;border:0;border-radius:5px;cursor:pointer}button:focus-visible,input:focus-visible{outline:3px solid #c96d30;outline-offset:3px}input{flex:1;min-width:150px}
.panels{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));gap:20px}section{background:white;padding:16px;border:1px solid #ccd3d0;border-radius:8px}h2{font-size:18px;margin:0 0 10px}canvas{width:100%;height:auto;border:1px solid #dce2df;display:block}output{font-variant-numeric:tabular-nums}.readout{min-height:54px;line-height:1.6;font-size:14px}small{display:block;line-height:1.6;color:#4d6267}.legend{font-size:14px}
</style><main><h1>Caster başlangıç açısı hareketi nasıl etkiliyor?</h1>
<p>Sentetik parametreli model; gerçek robot ölçümü veya doğrulaması değildir. Görünüm, Python çözümünün kayıtlı örneklerini oynatır. Her iki panel aynı metre ölçeğindedir.</p>
<div class="controls"><button id="play" type="button">Oynat</button><button id="restart" type="button">Başa dön</button><label for="scrub">Zaman</label><input id="scrub" type="range" min="0" step="0.001" value="0"><output id="time" for="scrub">0.000 s</output></div>
<div id="panels" class="panels"></div>
<p class="legend"><b>A</b>: tahrik aksı orta noktası · <b>B / COM</b>: gövde kütle merkezi · <b>P</b>: caster dönüş mafsalı. Siyah dikdörtgenler tekerleklerdir. Turuncu ok caster'ın yuvarlanma yönünü gösterir; kuvvet oku değildir.</p>
<small>theta: gövdenin dünya X eksenine göre açısı. phi: caster'ın gövdeye göre açısı. Caster'ın dünya açısı = theta + phi. Saat yönünün tersi pozitiftir. İlerleme çubuğunda seçilen zamana en yakın önceki kayıt çizilir; araya yapay hareket eklenmez. Gövde ve tekerlek boyutları parametre dosyasındandır.</small>
</main><script id="simulation-data" type="application/json">__SIMULATION_DATA__</script>
<script>
const data = JSON.parse(document.getElementById('simulation-data').textContent);
const cases = data.cases, scrub = document.getElementById('scrub'), play = document.getElementById('play');
const endTime = Math.max(...cases.map(c => c.samples[c.samples.length-1].time));
scrub.max = endTime;
let playing = false, elapsed = 0, lastFrame = null, animationFrame = null;
const margin = Math.max(...cases.map(c => Math.max(...c.geometry.chassis_size_m.slice(0,2), c.geometry.drive_track_m))) + 0.12;
let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
for (const c of cases) for (const s of c.samples) {xmin=Math.min(xmin,s.axle[0]);xmax=Math.max(xmax,s.axle[0]);ymin=Math.min(ymin,s.axle[1]);ymax=Math.max(ymax,s.axle[1]);}
const center = [(xmin+xmax)/2,(ymin+ymax)/2], span = Math.max(xmax-xmin,ymax-ymin)+2*margin;
const panels = cases.map((c,index) => {
 const section=document.createElement('section'), title=document.createElement('h2'), canvas=document.createElement('canvas'), readout=document.createElement('div');
 title.textContent=c.title;canvas.width=640;canvas.height=640;canvas.setAttribute('aria-label',c.title+' üstten robot hareketi');readout.className='readout';
 section.append(title,canvas,readout);document.getElementById('panels').append(section);
 return {c,canvas,readout,color:index===0?'#176a76':'#b24f31'};
});
function previousSample(samples,t){let low=0,high=samples.length-1;while(low<high){const mid=Math.ceil((low+high)/2);if(samples[mid].time<=t)low=mid;else high=mid-1;}return low;}
function draw(panel,t){
 const {c,canvas,readout,color}=panel,ctx=canvas.getContext('2d'),g=c.geometry;
 const index=previousSample(c.samples,t),s=c.samples[index],scale=560/span;
 function pixel(p){return [320+(p[0]-center[0])*scale,320-(p[1]-center[1])*scale];}
 function line(a,b,stroke,width=2){const p=pixel(a),q=pixel(b);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.stroke();}
 function point(p,label,fill){const q=pixel(p);ctx.beginPath();ctx.arc(...q,4,0,2*Math.PI);ctx.fillStyle=fill;ctx.fill();ctx.font='bold 15px system-ui';ctx.fillText(label,q[0]+7,q[1]-7);}
 function bodyPoint(x,y){return [s.axle[0]+Math.cos(s.theta)*x-Math.sin(s.theta)*y,s.axle[1]+Math.sin(s.theta)*x+Math.cos(s.theta)*y];}
 function rectangle(centerPoint,angle,length,width,fill,stroke){const corners=[[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]].map(([x,y])=>pixel([centerPoint[0]+Math.cos(angle)*x-Math.sin(angle)*y,centerPoint[1]+Math.sin(angle)*x+Math.cos(angle)*y]));ctx.beginPath();ctx.moveTo(...corners[0]);for(const p of corners.slice(1))ctx.lineTo(...p);ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=2;ctx.stroke();}
 ctx.clearRect(0,0,640,640);ctx.fillStyle='#fbfcfa';ctx.fillRect(0,0,640,640);
 // Dünya X/Y ızgarası metre olarak gösterilir; boyutlar iki panelde aynıdır.
 const grid=span>4?1:span>1?.25:.1;
 for(let x=Math.ceil((center[0]-span/2)/grid)*grid;x<center[0]+span/2;x+=grid)line([x,center[1]-span/2],[x,center[1]+span/2],'#e7ece9',1);
 for(let y=Math.ceil((center[1]-span/2)/grid)*grid;y<center[1]+span/2;y+=grid)line([center[0]-span/2,y],[center[0]+span/2,y],'#e7ece9',1);
 ctx.beginPath();for(let j=0;j<=index;j++){const p=pixel(c.samples[j].axle);if(j===0)ctx.moveTo(...p);else ctx.lineTo(...p);}ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
 rectangle(s.axle,s.theta,g.chassis_size_m[0],g.chassis_size_m[1],color+'22',color);
 line(bodyPoint(0,-g.drive_track_m/2),bodyPoint(0,g.drive_track_m/2),'#62777b',2);
 for(const side of [-1,1])rectangle(bodyPoint(0,side*g.drive_track_m/2),s.theta,2*g.drive_radius_m,g.drive_width_m,'#243238','#243238');
 line(s.pivot,s.caster,'#73502c',4);
 rectangle(s.caster,s.theta+s.phi,2*g.caster_radius_m,g.caster_width_m,'#243238','#243238');
 const worldAngle=s.theta+s.phi,arrowLength=g.caster_radius_m*3,tip=[s.caster[0]+arrowLength*Math.cos(worldAngle),s.caster[1]+arrowLength*Math.sin(worldAngle)];
 line(s.caster,tip,'#d66b24',3);
 for(const offset of [-.5,.5])line(tip,[tip[0]-.035*Math.cos(worldAngle+offset),tip[1]-.035*Math.sin(worldAngle+offset)],'#d66b24',3);
 point(s.axle,'A','#176a76');point(s.com,'B / COM','#95296b');point(s.pivot,'P','#73502c');
 ctx.fillStyle='#38545c';ctx.font='14px system-ui';ctx.fillText('Dünya X →  |  Y ↑',20,24);ctx.fillText('Izgara: '+grid+' m',20,46);
 line([center[0]-span*.40,center[1]-span*.43],[center[0]-span*.40+grid,center[1]-span*.43],'#183138',3);
 const deg=180/Math.PI;readout.textContent='Kayıt t = '+s.time.toFixed(3)+' s · theta = '+(s.theta*deg).toFixed(2)+'° · phi = '+(s.phi*deg).toFixed(2)+'° · A: X = '+s.axle[0].toFixed(3)+' m, Y = '+s.axle[1].toFixed(3)+' m';
}
function render(){scrub.value=elapsed;document.getElementById('time').textContent=elapsed.toFixed(3)+' s';panels.forEach(p=>draw(p,elapsed));}
function pause(){playing=false;play.textContent='Oynat';lastFrame=null;if(animationFrame!==null)cancelAnimationFrame(animationFrame);animationFrame=null;}
play.addEventListener('click',()=>{if(playing){pause();return;}if(elapsed>=endTime)elapsed=0;playing=true;lastFrame=null;play.textContent='Duraklat';animationFrame=requestAnimationFrame(tick);});
document.getElementById('restart').addEventListener('click',()=>{pause();elapsed=0;render();});
scrub.addEventListener('input',()=>{pause();elapsed=Number(scrub.value);render();});
function tick(now){if(!playing)return;if(lastFrame!==null)elapsed=Math.min(endTime,elapsed+(now-lastFrame)/1000);lastFrame=now;render();if(elapsed>=endTime)pause();else animationFrame=requestAnimationFrame(tick);}
render();
</script></html>'''
