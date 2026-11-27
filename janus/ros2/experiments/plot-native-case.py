#!/usr/bin/env python3
"""Plot measured native simulator state, commands and camera observations."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


EVENTS = {
    'brake':'fren', 'cut':'yayın kesildi', 'abort_injected':'iptal isteği',
    'regressive_odometry':'eski odom', 'clock_cut':'saat durdurma', 'clock_restore_service_ack':'geri dönüş alındı',
    'unauthorized_probe':'yetkisiz komut', 'recovery_ack_received':'yetki alındı', 'authorized_restart_input':'yeni komut',
    'camera_cut':'kamera kesildi', 'old_frame_rejected':'eski kare reddi', 'camera_restore':'kamera geri geldi',
    'qr_acquired':'QR bulundu', 'hide':'işaret kaybı', 'restore':'işaret döndü',
    'standoff':'yaklaşma duruşu', 'retry_budget_exhausted':'yeniden deneme sınırı',
    'optical_exposure_start':'pozlama arızası',
}


def load(folder):
    out = defaultdict(list)
    for line in (folder/'telemetry.jsonl').read_text().splitlines():
        row = json.loads(line)
        out[row['source']].append(row)
    return out


def plot(folder):
    rows = load(folder)
    execution = json.loads((folder/'execution.json').read_text())
    status = json.loads((folder/'acceptance.json').read_text())
    body, events = rows['truth'], rows['event']
    case = execution['case_id']
    initial = events[0]
    wall = case=='O08'
    axis_time = 'read_wall_s' if wall else 'read_time_s'
    base = initial[axis_time]
    axis_label = 'Duvar saati [s]' if wall else 'Simülasyon süresi [s]'

    fig, axes = plt.subplots(2,2,figsize=(11.0,7.7), layout='constrained')
    xy, scene, drive, side = axes.flat
    fig.patch.set_facecolor('#fbf8f1')
    for ax in axes.flat:
        ax.set_facecolor('#fffefa')
        ax.spines[['top','right']].set_visible(False)
        ax.grid(color='#e6e3d9', alpha=.75, linewidth=.6)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=8)
    x = [row['x_m'] for row in body]
    y = [row['y_m'] for row in body]
    xy.plot(x,y,color='#245a7c',linewidth=1.9,zorder=3)
    xy.scatter([x[0],x[-1]], [y[0],y[-1]], color=['#71846b','#b45d38'],zorder=4, s=29)
    xy.set(title='Bağımsız Gazebo aks izi', xlabel='Dünya X [m]', ylabel='Dünya Y [m]')
    xy.set_aspect('equal', adjustable='datalim')
    def draw_series(ax, source, name, color, label, *, dashed=False):
        usable = [row for row in rows[source] if row[axis_time] is not None and row[axis_time]>=base]
        stamps = [row[axis_time]-base for row in usable]
        vals = [row[name] for row in usable]
        ax.plot(stamps,vals,color=color,label=label,linewidth=1.2,linestyle='--' if dashed else '-')

    draw_series(drive,'truth','v_mps','#245a7c','Ölçülen aks hızı')
    draw_series(drive,'guard_output','v_mps','#b45d38','Guard çıkışı',dashed=True)
    drive.axhspan(-.02,.02,color='#71846b',alpha=.14)
    drive.set(title='İleri hareket ve duruş', ylabel='Hız [m/s]',xlabel=axis_label)
    drive.legend(loc='center right',frameon=True,facecolor='#fffefa',edgecolor='none',fontsize=8)
    marks = [row for row in events if row['event'] in EVENTS and row[axis_time] is not None]
    for index, event in enumerate(marks[:6],1):
        stamp=event[axis_time]-base
        drive.axvline(stamp,color='#66736b',linewidth=.8,alpha=.65)
        drive.text(stamp,.97 if index%2==1 else .81,str(index),transform=drive.get_xaxis_transform(),
                   size=8,ha='center',va='top',color='#344a3b',
                   bbox={'boxstyle':'circle,pad=.18','facecolor':'#f4f0e4','edgecolor':'#aab7a2','linewidth':.7})
    if marks:
        text='    '.join(f'{i} {EVENTS[row["event"]]}' for i,row in enumerate(marks[:6],1))
        drive.text(.002,-.28,text,transform=drive.transAxes,ha='left',va='top',fontsize=7,wrap=True)

    for image_name in ('loss-rendered.png','standoff-rendered.png','steady-rendered.png','start-rendered.png'):
        if (folder/image_name).is_file():
            scene.imshow(mpimg.imread(folder/image_name))
            scene.set_title('Gazebo ön kamerası · modelin render karesi')
            break
    scene.axis('off')
    if rows['qr']:
        valid = [row for row in rows['qr'] if row['valid'] and row['normalized_error'] is not None
                 and row[axis_time] is not None and row[axis_time]>=base]
        invalid = [row for row in rows['qr'] if not row['valid'] and row[axis_time] is not None and row[axis_time]>=base]
        side.axhspan(-.02,.02,color='#71846b',alpha=.16,label='İç bant ±%2')
        side.scatter([row[axis_time]-base for row in valid],[row['normalized_error'] for row in valid],
                     s=10,color='#245a7c',label='Geçerli gözlem')
        side.scatter([row[axis_time]-base for row in invalid],[.95]*len(invalid),s=12,marker='x',
                     transform=side.get_xaxis_transform(),color='#b45d38',label='Ret veya görünmez')
        side.set(title='Gerçek renderdan QR merkezlemesi', ylabel='Merkez hatası / genişlik', xlabel=axis_label)
    else:
        draw_series(side,'joint','left_wheel_radps','#245a7c','Sol teker')
        draw_series(side,'joint','right_wheel_radps','#b45d38','Sağ teker',dashed=True)
        side.set(title='Native eklem hızı',ylabel='Açısal hız [rad/s]',xlabel=axis_label)
    side.legend(loc='best',frameon=False,fontsize=8)
    case_label = 'Doğrulanan koşu' if status['status']=='passed' else 'Tanı / açık ölçüt'
    fig.suptitle(f'{case}  ·  {case_label}     ROS 2 Jazzy / Gazebo Harmonic',fontsize=17, color='#283b45',
                 fontweight='bold',x=.015,ha='left')
    fig.savefig(folder/'native-evidence.png',dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    for folder in parser.parse_args().directories:
        plot(folder)


if __name__ == '__main__':
    main()
