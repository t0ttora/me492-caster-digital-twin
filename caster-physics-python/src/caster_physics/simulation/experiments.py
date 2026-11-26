"""OKUMA 10 — Deneyde neyi değiştiriyoruz, neyi sabit tutuyoruz?

Burada fizik denklemi yoktur. Başlangıç, tork programı ve karşılaştırma
koşulları vardır. İsimler MATLAB'ın kayıtlı deney kimlikleriyle aynıdır.
"""
from dataclasses import dataclass, replace
import numpy as np

from ..state import State, Torques
from ..physics.geometry import geometry_at


@dataclass(frozen=True)
class Experiment:
    name: str
    description_tr: str
    duration_s: float
    initial_caster_angle_rad: float
    caster_contact_enabled: bool = True
    step_multiplier: float = 1.0
    initial_condition: str = "rest"


EXPERIMENTS = (
    Experiment("aligned_forward", "İleriye hizalı caster ile aynı motor torkları", 4, 0),
    Experiment("misaligned_forward", "Başlangıçta 45 derece caster, aynı motor torkları", 4, np.pi/4),
    Experiment("mirror_misaligned", "Eksi 45 derece: işaret simetrisi", 4, -np.pi/4),
    Experiment("caster_contact_disabled", "Teğetsel caster etkileri kapalı; kütle ve destek duruyor", 4, np.pi/4, False),
    Experiment("unequal_torque", "Sağ tekerlek torku daha yüksek", 4, 0),
    Experiment("reversal_perturbed", "1 derece başlangıç, ileri sonra geri tork", 7, np.pi/180),
    Experiment("zero_rest", "Duran robot, sıfır motor torku", 2, np.pi/4),
    Experiment("passive_coast", "Hareketli başlangıç, sıfır torkta sönüm", 5, np.pi/4, initial_condition="coast"),
    Experiment("conservative_free", "Sürtünmesiz enerji/momentum deneyi", 3, .6, False, initial_condition="free"),
    Experiment("misaligned_fine", "45 derece, yarım çözüm adımı", 4, np.pi/4, step_multiplier=.5),
    Experiment("misaligned_finer", "45 derece, çeyrek çözüm adımı", 4, np.pi/4, step_multiplier=.25),
    Experiment("reversal_no_swivel_resistance", "Aynı geri tork, yatak ve bore direnci yok", 7, np.pi/180),
)


def experiment_by_name(name):
    for experiment in EXPERIMENTS:
        if experiment.name == name:
            return experiment
    raise ValueError(f"Bilinmeyen deney: {name}")


def parameters_for_experiment(parameters, experiment):
    """Ablation fizik kaldırma karşılaştırmasıdır; ana parametre dosyasını değiştirmez."""
    dynamics = parameters.dynamics
    if experiment.initial_condition == "free":
        dynamics = replace(dynamics,
            drive_friction_coefficient=0, caster_friction_coefficient=0,
            drive_long_slip_damping_Nspm=0, drive_lat_slip_damping_Nspm=0,
            caster_long_slip_damping_Nspm=0, caster_lat_slip_damping_Nspm=0,
            drive_rolling_resistance_coefficient=0, caster_rolling_resistance_coefficient=0,
            drive_roll_viscous_NmspRad=0, caster_roll_viscous_NmspRad=0,
            swivel_bearing_viscous_NmspRad=0, swivel_bearing_coulomb_Nm=0,
            caster_bore_effective_radius_m=0)
    if experiment.name == "reversal_no_swivel_resistance":
        dynamics = replace(dynamics, swivel_bearing_viscous_NmspRad=0,
                           swivel_bearing_coulomb_Nm=0, caster_bore_effective_radius_m=0)
    return replace(parameters, dynamics=dynamics)


def initial_state(parameters, experiment):
    state = replace(State.at_rest(parameters), caster_angle_relative_rad=experiment.initial_caster_angle_rad)
    if experiment.initial_condition == "coast":
        wheel_rate = .35/parameters.geometry.drive_radius_m
        state = replace(state, body_velocity_x_mps=.35, body_yaw_rate_radps=.15,
                        caster_relative_rate_radps=-.1, left_wheel_rate_radps=wheel_rate,
                        right_wheel_rate_radps=wheel_rate)
        geometry = geometry_at(state, parameters)
        caster_speed = geometry.caster_forward @ (geometry.caster_velocity_jacobian @ state.velocities())
        state = replace(state, caster_roll_rate_radps=caster_speed/parameters.geometry.caster_radius_m)
    elif experiment.initial_condition == "free":
        state = replace(state, body_velocity_x_mps=.15, body_velocity_y_mps=-.05,
                        body_yaw_rate_radps=.3, caster_relative_rate_radps=-.4,
                        left_wheel_rate_radps=2, right_wheel_rate_radps=-1, caster_roll_rate_radps=3)
    return state


def sampled_torques(sample_index, parameters, experiment):
    """Tork yalnız komut örneklerinde değişir; sürekli rampa değildir."""
    time_s = sample_index*parameters.dynamics.command_sample_dt_s
    common = .25*min(time_s/.2, 1)
    if time_s >= 1:
        common = .06
    left, right = common, common
    if experiment.name == "unequal_torque":
        left, right = .15*min(time_s/.2, 1), .25*min(time_s/.2, 1)
        if time_s >= 1.5:
            left, right = .02, .06
    elif experiment.name.startswith("reversal_") and time_s >= 1.5:
        left = right = -.25
    elif experiment.name in {"zero_rest", "passive_coast", "conservative_free"}:
        left = right = 0
    return Torques(left, right)
