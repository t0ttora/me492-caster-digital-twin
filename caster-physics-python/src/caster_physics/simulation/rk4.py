"""OKUMA 11 — İvmeden hıza, hızdan konuma nasıl ilerliyoruz?

Klasik sabit adımlı RK4, aynı adımda dört türev hesabını birleştirir.
MATLAB ode4'teki AYRIK kaynakla aynı şekilde komut dört kademede sabittir.
Çözüm adımı, komut örnekleme aralığı ve CSV kayıt aralığı farklı kavramlardır.
"""
from dataclasses import dataclass
import numpy as np

from ..state import State
from ..physics.motion import evaluate_motion
from ..physics.energy import kinetic_energy
from .experiments import initial_state, parameters_for_experiment, sampled_torques
from .records import result_record


@dataclass
class SimulationResult:
    experiment: object
    parameters: object
    states: np.ndarray
    records: list
    step_s: float

    def column(self, name):
        return np.array([record[name] for record in self.records])


def rk4_step(vector, step_s, derivative):
    """Fizik türevini dört noktada ölç; ara komut değiştirme."""
    first = derivative(vector)
    second = derivative(vector+step_s*first/2)
    third = derivative(vector+step_s*second/2)
    fourth = derivative(vector+step_s*third)
    return vector+step_s*(first+2*second+2*third+fourth)/6


def simulate(parameters, experiment):
    parameters = parameters_for_experiment(parameters, experiment)
    d = parameters.dynamics
    step_s = d.solver_step_s*experiment.step_multiplier
    if not np.isfinite(step_s) or step_s <= 0 or not np.isfinite(experiment.duration_s) or experiment.duration_s <= 0:
        raise ValueError("Deney süresi ve adımı sonlu/pozitif olmalı.")
    ratios = [experiment.duration_s/step_s, d.command_sample_dt_s/step_s, d.result_sample_dt_s/step_s]
    if any(not np.isfinite(value) or value < 1 or abs(value-round(value)) >= 1e-8 for value in ratios):
        raise ValueError("Süre, komut ve kayıt aralığı çözüm adımının tam katı olmalı.")
    steps, steps_per_command, steps_per_record = [round(value) for value in ratios]
    start = initial_state(parameters, experiment)
    vector = start.as_vector()
    initial_energy = kinetic_energy(start, parameters)
    saved_states, records = [], []
    for step_index in range(steps+1):
        # Tam sayı indeks: kayan noktalı floor(t/0.01) sınır hatalarını önler.
        torques = sampled_torques(step_index//steps_per_command, parameters, experiment)
        if step_index % steps_per_record == 0 or step_index == steps:
            state = State.from_vector(vector)
            evaluation = evaluate_motion(state, torques, parameters, experiment.caster_contact_enabled)
            records.append(result_record(step_index*step_s, state, torques, parameters, evaluation, initial_energy))
            saved_states.append(vector.copy())
        if step_index == steps:
            break

        def derivative(at_vector):
            return evaluate_motion(State.from_vector(at_vector), torques, parameters,
                                   experiment.caster_contact_enabled).derivative

        vector = rk4_step(vector, step_s, derivative)
    return SimulationResult(experiment, parameters, np.array(saved_states), records, step_s)
