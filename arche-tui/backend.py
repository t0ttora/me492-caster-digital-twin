"""Presentation adapter: no changes to either preserved physics implementation."""
from pathlib import Path
from dataclasses import asdict, replace
from unittest.mock import patch
from contextlib import nullcontext, contextmanager
from collections import deque
import time
import csv, json, sys, math, hashlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'caster-physics-python/src'),str(ROOT/'caster-physics-python-v2')]
from caster_physics.parameters import load_parameters, validate_parameters, Parameters, Geometry, Dynamics
from caster_physics.state import State, Torques
from caster_physics.simulation import rk4
from caster_physics.simulation.experiments import EXPERIMENTS, experiment_by_name, initial_state
import twin

VERSIONS={'V1':'Mechanical baseline: predict motion from wheel torques. Original inertias and MATLAB-parity kernel are preserved.',
          'V2':'Trajectory tracking: the V1 mechanical kernel with corrected wheel inertias, wheel PI, terminal pose recovery and reference paths.'}
OUTPUT=ROOT/'arche-tui/outputs'
HELP={
 'drive_radius_m':'Driven-wheel radius [m]. Converts wheel rotation to linear travel.',
 'drive_track_m':'Distance between left/right drive-wheel centres [m].',
 'caster_radius_m':'Caster rolling radius [m]. Edit spin inertia separately when changing hardware.',
 'caster_trail_m':'Swivel pivot to caster axle distance [m].',
 'caster_pivot_m':'Caster pivot [x,y,z] in body frame [m]; x forward, y left, z up.',
 'chassis_size_m':'Chassis dimensions [length,width,height], metres.',
 'drive_width_m':'Drive wheel width [m]; representative geometry, not a tyre compliance model.',
 'caster_width_m':'Caster wheel width [m]; no resolved 3D tyre deformation in this planar kernel.',
 'body_lumped_mass_kg':'Lumped main-body mass including the modelled drive assembly [kg].',
 'caster_lumped_mass_kg':'Lumped caster assembly mass [kg].',
 'body_com_m':'Body centre of mass [x,y,z] relative to drive-axle origin [m]. Must lie in the support region.',
 'body_yaw_inertia_kgm2':'Main-body yaw inertia about its centre of mass [kg m^2].',
 'caster_swivel_inertia_com_kgm2':'Caster yaw inertia about its own COM [kg m^2], not wheel spin inertia.',
 'drive_roll_inertia_kgm2':'Per-drive-wheel spin inertia [kg m^2]. Frozen V1=.0045; cylinder-aligned V2=.001125.',
 'caster_roll_inertia_kgm2':'Caster wheel spin inertia [kg m^2]. Frozen V1=.00045; aligned V2=.00032.',
 'gravity_mps2':'Gravitational acceleration [m/s^2].',
 'solver_step_s':'RK4 integration step [s]. Smaller is slower; sample periods must be integer multiples.',
 'command_sample_dt_s':'Controller/torque update period [s], held constant between updates.',
 'result_sample_dt_s':'CSV recording period [s]. Controls data and plotting resolution.',
 'preset':'V1 experiment. Choosing it loads its initial state, angle, duration and documented ablation.',
 'torque_mode':'preset = original sampled torque program; constant = the left/right values below.',
 'left_torque_Nm':'Constant left wheel axle torque [N m], used only in constant mode.',
 'right_torque_Nm':'Constant right wheel axle torque [N m], used only in constant mode.',
 'duration_s':'V1 simulation duration [s]. Must be an integer number of solver steps.',
 'maneuver':'Reference path: straight, arc turn, forward/reverse, or S turn.',
 'profile':'baseline timing or quintic smooth timing. Smooth duration is 1.875 times longer at equal peak reference speed.',
 'surface':'Choosing a surface preset updates its physics fields now. Subsequent edits take precedence.',
 'design':'Choosing a geometry preset updates affected fields now; radius presets scale caster spin inertia at fixed mass.',
 'segment_distance_m':'Distance magnitude of each path segment [m]. Reversal keeps its negative return segment.',
 'segment_yaw_rad':'Arc/S-turn heading change magnitude [rad]; straight/reversal remain zero.',
 'segment_duration_s':'Baseline duration of each segment [s]. Smooth stretches this by 1.875.',
 'terminal_s':'Additional observation after the reference ends [s]. Completion also requires a 0.25 s dwell.',
 'mode':'auto derives gains from the existing selected pole/time targets; manual uses the eight editable gains below.',
 'tracking_x':'Longitudinal tracking gain [1/s]. Automatic target time constant: 0.5 s.',
 'tracking_y':'Lateral tracking gain [1/m^2]. Automatic target: critical damping, 1 rad/s at configured straight speed.',
 'tracking_heading':'Heading tracking gain [1/s]. Automatic value: 2.',
 'wheel_p':'Wheel-speed proportional gain [N m s/rad]. Auto derives from poles .8/.5 and current inertia/sample time.',
 'wheel_i':'Wheel-speed integral gain [N m/rad]. Auto derives from poles .8/.5; anti-windup remains active.',
 'terminal_rho':'Terminal position gain [1/s]; auto 2.',
 'terminal_alpha':'Terminal bearing gain [1/s]; auto 3.',
 'terminal_heading':'Final heading gain [1/s]; auto 3.',
}

def help_for(key):
    if key in HELP:return HELP[key]
    if key.endswith('_friction_coefficient'):return 'Dimensionless ground friction limit for this wheel set. Synthetic, not an identified physical surface.'
    if 'slip_damping' in key:return 'Slip-velocity force regularisation [N s/m]. Higher values resist wheel sliding more strongly.'
    if 'rolling_resistance_coefficient' in key:return 'Dimensionless rolling-resistance coefficient; contributes a resisting wheel torque.'
    if 'viscous' in key:return 'Viscous resistance coefficient [N m s/rad], opposing the relevant rolling or swivel rate.'
    if 'regularization' in key:return 'Regularisation angular speed [rad/s]. Sets the smooth transition around zero velocity.'
    if key=='swivel_bearing_coulomb_Nm':return 'Coulomb bearing-resistance torque [N m], smoothed near zero swivel rate.'
    if key=='caster_bore_effective_radius_m':return 'Effective caster contact-patch radius [m] used in the phenomenological bore torque.'
    if key.startswith('body_') or key.startswith('caster_') or 'wheel_rate' in key:return key.replace('_',' ')+'. Initial state, SI units; angles in radians. Body x/y describe COM, not the axle.'
    return key.replace('_',' ')+'. Editable model input; all values use SI units.'

def parameters(config):
    def cast(data):return {k:tuple(v) if isinstance(v,list) else v for k,v in data.items()}
    return Parameters(Geometry(**cast(config['geometry'])),Dynamics(**cast(config['dynamics'])))

def defaults(version):
    p=load_parameters(ROOT/('caster-physics-python' if version=='V1' else 'caster-physics-python-v2')/'parameters')
    c={'version':version,**json.loads(json.dumps(asdict(p))),
       'experiment':({'preset':'aligned_forward','torque_mode':'preset','left_torque_Nm':.1,'right_torque_Nm':.1,'duration_s':4.} if version=='V1' else
       {'maneuver':'straight','profile':'baseline','surface':'reference_synthetic','design':'nominal','segment_distance_m':.30,'segment_yaw_rad':math.pi/3,'segment_duration_s':2.,'terminal_s':8.}),
       'initial':asdict(replace(State.at_rest(p),caster_angle_relative_rad=0 if version=='V1' else math.pi/4)),
       'gains':{'mode':'auto',**twin.controller_gains(p)}}
    return c

def select_preset(c,key,value):
    e=c['experiment'];e[key]=value
    if key=='preset':
        ex=experiment_by_name(value);e['duration_s']=ex.duration_s;c['initial']=asdict(initial_state(parameters(c),ex))
    elif key=='surface':
        base=defaults('V2')['dynamics']
        for name in set().union(*(set(x) for x in twin.SURFACES.values())):c['dynamics'][name]=base[name]
        c['dynamics'].update(twin.SURFACES[value])
    elif key=='design':
        # Apply against current values, with the same fixed-mass radius scaling as V2.
        changes=twin.DESIGNS[value]
        if value=='nominal':
            base=defaults('V2')
            for name in ['caster_trail_m','caster_radius_m']:c['geometry'][name]=base['geometry'][name]
            for name in ['caster_roll_inertia_kgm2','swivel_bearing_viscous_NmspRad']:c['dynamics'][name]=base['dynamics'][name]
        for name,v in changes.items():
            target=c['geometry'] if name in c['geometry'] else c['dynamics']
            if name=='caster_radius_m':c['dynamics']['caster_roll_inertia_kgm2']*=(v/target[name])**2
            target[name]=v
    return c

def maneuvers(c):
    e=c['experiment']
    return {name:tuple(twin.Segment(s.distance_m/.30*e['segment_distance_m'],s.yaw_change_rad/(math.pi/3)*e['segment_yaw_rad'],e['segment_duration_s']) for s in segments) for name,segments in twin.MANEUVERS.items()}

def active_gains(c):
    if c['gains']['mode']=='manual':return {k:v for k,v in c['gains'].items() if k!='mode'}
    with patch.object(twin,'MANEUVERS',maneuvers(c)):return twin.controller_gains(parameters(c))

def validate(c):
    if c['version'] not in VERSIONS:raise ValueError('Choose V1 or V2.')
    p=parameters(c);validate_parameters(p)
    State.from_vector(State(**c['initial']).as_vector())
    if c['initial']['motor_work_J']!=0 or c['initial']['dissipated_energy_J']!=0:raise ValueError('Energy accounts must start at zero.')
    e=c['experiment'];h=p.dynamics.solver_step_s
    if c['version']=='V1':
        ex=experiment_by_name(e['preset']);h*=ex.step_multiplier;duration=float(e['duration_s'])
        if e['torque_mode'] not in ['preset','constant']:raise ValueError('Invalid torque mode.')
        Torques(e['left_torque_Nm'],e['right_torque_Nm'])
    else:
        if e['maneuver'] not in twin.MANEUVERS or e['profile'] not in twin.PROFILES:raise ValueError('Invalid path/profile.')
        for name in ['segment_duration_s','segment_distance_m']:
            if not math.isfinite(e[name]) or e[name]<=0:raise ValueError(name+' must be positive and finite.')
        if not math.isfinite(e['segment_yaw_rad']) or not math.isfinite(e['terminal_s']) or e['terminal_s']<0:raise ValueError('Invalid reference settings.')
        duration=e['segment_duration_s']*len(twin.MANEUVERS[e['maneuver']])*(1.875 if e['profile']=='smooth' else 1)+e['terminal_s']
        if c['gains']['mode'] not in ['auto','manual']:raise ValueError('Invalid gain mode.')
        if any(not math.isfinite(v) or v<=0 for k,v in c['gains'].items() if k!='mode'):raise ValueError('Gains must be positive finite numbers.')
    for v in [duration,p.dynamics.command_sample_dt_s,p.dynamics.result_sample_dt_s]:
        if not math.isfinite(v) or v<=0 or abs(v/h-round(v/h))>1e-8:raise ValueError('Duration and sample periods must be integer multiples of the effective solver step.')
    if duration<2*p.dynamics.result_sample_dt_s:raise ValueError('Use at least two result intervals (three records).')
    return p,duration

def read_rows(path):
    with Path(path).open(newline='') as f:rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    if not rows or 'time_s' not in rows[0] or any(not math.isfinite(v) for r in rows for v in r.values()):raise ValueError('Expected finite numeric simulation CSV with time_s.')
    if any(b['time_s']<=a['time_s'] for a,b in zip(rows,rows[1:])):raise ValueError('Timestamps must increase strictly.')
    return rows

@contextmanager
def stream_progress(version, out, duration):
    """Observe recorded samples; never alter derivatives, states or integration."""
    module = rk4 if version == 'V1' else twin
    original = module.result_record
    recent = deque(maxlen=600)
    count, last_write = 0, -1.0
    def flush():
        if not recent:return
        payload = {'version': version, 'time_s': recent[-1]['time_s'],
                   'progress': min(1., recent[-1]['time_s']/duration),
                   'sample_count': count, 'rows': list(recent)}
        temporary = out/'live.json.tmp'
        temporary.write_text(json.dumps(payload, allow_nan=False))
        temporary.replace(out/'live.json')
    def observed(*args, **kwargs):
        nonlocal count, last_write
        row = original(*args, **kwargs)
        recent.append(row);count += 1
        now = time.monotonic()
        if now-last_write >= .25:
            flush();last_write = now
        return row
    try:
        with patch.object(module, 'result_record', observed):yield
    finally:
        flush()

def execute(c,out,live=False):
    p,duration=validate(c);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    # The caller allocates a fresh run directory; never overwrite an existing result.
    config_path=out/'config.json'
    with config_path.open('x') as f:json.dump(c,f,indent=2,allow_nan=False)
    preservation=twin.verify_v1()
    with stream_progress(c['version'],out,duration) if live else nullcontext():
        rows, effective, metrics = simulate_config(c,p,duration)
    if any(not math.isfinite(v) for row in rows for v in row.values()):raise ValueError('Nonfinite simulation output; reduce step or review parameters.')
    metrics.update({'version':c['version'],'samples':len(rows),'duration_s':rows[-1]['time_s'],'final_x_m':rows[-1]['x_m'],'final_y_m':rows[-1]['y_m'],'final_yaw_rad':rows[-1]['yaw_rad'],
                    'energy_balance_max_J':max(abs(r['energy_balance_error_J']) for r in rows),'torque_peak_Nm':max(max(abs(r['left_drive_torque_Nm']),abs(r['right_drive_torque_Nm'])) for r in rows),
                    'v1_preserved':preservation['passed'],'physical_robot_validated':False})
    with (out/'results.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    for name,data in [('summary.json',metrics),('effective_parameters.json',effective)]:
        with (out/name).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
    return out

def simulate_config(c,p,duration):
    if c['version']=='V1':
        e=c['experiment'];ex=replace(experiment_by_name(e['preset']),duration_s=duration,initial_caster_angle_rad=c['initial']['caster_angle_relative_rad'])
        torque_patch=patch.object(rk4,'sampled_torques',lambda *_:Torques(e['left_torque_Nm'],e['right_torque_Nm'])) if e['torque_mode']=='constant' else nullcontext()
        with torque_patch,patch.object(rk4,'initial_state',lambda *_:State(**c['initial'])):result=rk4.simulate(p,ex)
        rows=result.records;effective=asdict(result.parameters);metrics={}
    else:
        e=c['experiment'];manual={k:v for k,v in c['gains'].items() if k!='mode'}
        with patch.object(twin,'MANEUVERS',maneuvers(c)):
            gains=twin.controller_gains(p) if c['gains']['mode']=='auto' else manual
            with patch.object(twin,'controller_gains',lambda _:gains):
                ctrl=twin.TrackingController(p,e['maneuver'],e['profile'])
                rows=twin.integrate(p,duration,State(**c['initial']),ctrl,lambda t:twin.reference_at(t,e['maneuver'],e['profile']))
                metrics=twin.summarize(rows,p,e['maneuver'],e['profile'])
        metrics['active_controller_gains']=gains;effective=asdict(p)
    return rows, effective, metrics

def ascii_plot(xs,ys,width,height):
    width=max(2,width);height=max(2,height);grid=[[' ']*width for _ in range(height)]
    if not xs:return [''.join(r) for r in grid]
    lo,hi=min(xs),max(xs);bottom,top=min(ys),max(ys)
    for x,y in zip(xs,ys):
        col=round((x-lo)/(hi-lo or 1)*(width-1));row=height-1-round((y-bottom)/(top-bottom or 1)*(height-1))
        grid[max(0,min(height-1,row))][max(0,min(width-1,col))]='*'
    return [''.join(r) for r in grid]

if __name__=='__main__':
    try:
        config=json.loads(Path(sys.argv[1]).read_text());execute(config,Path(sys.argv[2]),live=True);print('DONE',flush=True)
    except Exception as error:
        import traceback
        traceback.print_exc();sys.exit(1)


def result_help(key):
    exact={
        'time_s':'Elapsed simulation time [s]. Playback shows saved samples, not newly simulated motion.',
        'x_m':'World X coordinate of drive-axle midpoint A [m], not the integrated body centre of mass.',
        'y_m':'World Y coordinate of drive-axle midpoint A [m]. X forward and Y left at zero yaw.',
        'yaw_rad':'Body heading in the world frame [rad], counter-clockwise positive.',
        'caster_angle_body_rad':'Caster swivel angle relative to the chassis [rad]. Zero means aligned with body forward.',
        'caster_angle_world_rad':'Absolute caster heading [rad] = body yaw + relative swivel angle.',
        'energy_balance_error_J':'Numerical residual T(t) + D(t) - W(t) - T(0) [J]. Small values check accounting, not physical calibration.',
        'motor_work_J':'Accumulated signed mechanical work at wheel axles [J]; not battery energy consumption.',
        'dissipated_energy_J':'Accumulated modelled rolling, slipping, bearing and bore losses [J].',
        'kinetic_energy_J':'Total modelled translational and rotational kinetic energy [J].',
        'motor_power_W':'Instantaneous axle mechanical power tau_L*omega_L + tau_R*omega_R [W].',
        'dissipation_power_W':'Instantaneous passive mechanical loss [W]; negative values beyond roundoff need investigation.',
        'completion_time_s':'Start of a verified 0.25 s terminal dwell: <=2 cm, <=0.05 rad, low planar/yaw speed. None means not reached.',
        'completion_within_original_1s_window_s':'Completion checked with the original one-second terminal window, before allowing longer recovery.',
        'controller_phase_code':'0: moving-reference tracking; 1: terminal position recovery; 2: final heading alignment.',
    }
    if key in exact:return exact[key]
    if 'rmse' in key:return 'Root mean square error against the named reference; position/path in metres and yaw in radians. Timing and spatial path errors are different metrics.'
    if key.startswith('ref_'):return 'Commanded reference trajectory, not a measured physical-robot state. Units are in the channel name.'
    if key.startswith('m0_encoder'):return 'Ideal no-slip wheel odometry calculated from the same wheel rates; not independent ground truth.'
    if key.startswith('normal_'):return 'Quasi-static vertical support force [N]. This planar model does not simulate 3D impact/load transfer.'
    if 'slip' in key:return 'Wheel-ground longitudinal or lateral slip velocity [m/s], measured in the relevant wheel frame.'
    if '_f' in key and key.endswith('_N'):return 'Ground contact force component in world coordinates [N].'
    if 'torque' in key:return 'Mechanical torque at the stated wheel axle or resistance axis [N m].'
    if 'rate_radps' in key:return 'Angular rate [rad/s]; caster relative rate is measured relative to the chassis.'
    if 'velocity_mps' in key:return 'Velocity [m/s] in the stated frame; body-long/body-lat channels use chassis axes.'
    if 'body_com_' in key:return 'Integrated main-body centre-of-mass world position [m], distinct from axle position A.'
    return key.replace('_',' ')+'. Recorded model quantity; units appear in the channel name. Synthetic data.'
