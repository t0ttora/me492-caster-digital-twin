"""OKUMA 2 — İsimli, birimli parametreler. p[27] gibi gizli indeksler yok.

Geometri ile kütle/sürtünme değerleri ayrıdır. Değerlerin tamamı sentetiktir.
"""
from dataclasses import dataclass, fields
from pathlib import Path
import json

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Geometry:
    drive_radius_m: float
    drive_track_m: float
    caster_radius_m: float
    caster_trail_m: float
    caster_pivot_m: tuple
    chassis_size_m: tuple
    drive_width_m: float
    caster_width_m: float


@dataclass(frozen=True)
class Dynamics:
    body_lumped_mass_kg: float
    caster_lumped_mass_kg: float
    body_com_m: tuple
    body_yaw_inertia_kgm2: float
    caster_swivel_inertia_com_kgm2: float
    drive_roll_inertia_kgm2: float
    caster_roll_inertia_kgm2: float
    gravity_mps2: float
    drive_friction_coefficient: float
    caster_friction_coefficient: float
    drive_long_slip_damping_Nspm: float
    drive_lat_slip_damping_Nspm: float
    caster_long_slip_damping_Nspm: float
    caster_lat_slip_damping_Nspm: float
    drive_rolling_resistance_coefficient: float
    caster_rolling_resistance_coefficient: float
    drive_roll_viscous_NmspRad: float
    caster_roll_viscous_NmspRad: float
    swivel_bearing_viscous_NmspRad: float
    swivel_bearing_coulomb_Nm: float
    rolling_regularization_radps: float
    swivel_regularization_radps: float
    caster_bore_effective_radius_m: float
    bore_regularization_radps: float
    solver_step_s: float
    command_sample_dt_s: float
    result_sample_dt_s: float


@dataclass(frozen=True)
class Parameters:
    geometry: Geometry
    dynamics: Dynamics


def load_parameters(directory=None):
    """Çalışma kopyasını oku; MATLAB klasöründen parametre değiştirme."""
    directory = Path(directory) if directory is not None else PROJECT_ROOT / "parameters"
    geometry_data = json.loads((directory / "geometry_si.json").read_text())
    dynamics_data = json.loads((directory / "dynamics_si.json").read_text())
    for data in (geometry_data, dynamics_data):
        if data["units"] != "SI" or data["parameter_status"] != "provisional_synthetic":
            raise ValueError("Bu sürüm yalnız SI ve provisional_synthetic parametreleri kabul eder.")
    def named_values(kind, data):
        return kind(**{field.name: tuple(data[field.name]) if isinstance(data[field.name], list)
                      else data[field.name] for field in fields(kind)})
    parameters = Parameters(named_values(Geometry, geometry_data), named_values(Dynamics, dynamics_data))
    validate_parameters(parameters)
    return parameters


def validate_parameters(parameters):
    """Negatif yükü kırpmak yerine fiziksel olarak desteklenmeyen robotu reddet."""
    g, d = parameters.geometry, parameters.dynamics
    for group in (g, d):
        for field in fields(group):
            if not np.all(np.isfinite(getattr(group, field.name))):
                raise ValueError(f"{field.name} sonlu olmalı.")
    if len(g.caster_pivot_m) != 3 or len(g.chassis_size_m) != 3 or len(d.body_com_m) != 3:
        raise ValueError("Geometri ve COM vektörleri üç bileşenli olmalı.")
    if min(g.drive_radius_m, g.drive_track_m, g.caster_radius_m, g.caster_trail_m,
           g.drive_width_m, g.caster_width_m, *g.chassis_size_m) <= 0:
        raise ValueError("Boyutlar pozitif olmalı.")
    if g.caster_pivot_m[0] + g.caster_trail_m >= 0:
        raise ValueError("Bu modelde caster tahrik aksının arkasında kalmalı.")
    positive = ["body_lumped_mass_kg", "caster_lumped_mass_kg", "body_yaw_inertia_kgm2",
                "caster_swivel_inertia_com_kgm2", "drive_roll_inertia_kgm2", "caster_roll_inertia_kgm2",
                "gravity_mps2", "drive_long_slip_damping_Nspm", "drive_lat_slip_damping_Nspm",
                "caster_long_slip_damping_Nspm", "caster_lat_slip_damping_Nspm",
                "rolling_regularization_radps", "swivel_regularization_radps", "bore_regularization_radps",
                "solver_step_s", "command_sample_dt_s", "result_sample_dt_s"]
    for name in positive:
        if getattr(d, name) <= 0:
            raise ValueError(f"{name} pozitif olmalı.")
    for field in fields(d):
        if field.name not in positive and field.name != "body_com_m" and getattr(d, field.name) < 0:
            raise ValueError(f"{field.name} negatif olamaz.")
    for interval in [d.command_sample_dt_s, d.result_sample_dt_s]:
        if abs(interval/d.solver_step_s-round(interval/d.solver_step_s)) >= 1e-9:
            raise ValueError("Komut ve kayıt aralıkları çözüm adımının tam katı olmalı.")
    from .state import State
    from .physics.support_loads import support_loads
    for angle in np.linspace(-np.pi, np.pi, 361):
        support_loads(State(caster_angle_relative_rad=angle), parameters)
