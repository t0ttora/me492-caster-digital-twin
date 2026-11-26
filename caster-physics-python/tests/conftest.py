from functools import lru_cache

import pytest

from caster_physics.parameters import load_parameters
from caster_physics.simulation.experiments import experiment_by_name
from caster_physics.simulation.rk4 import simulate


@pytest.fixture(scope="session")
def parameters():
    return load_parameters()


@pytest.fixture(scope="session")
def run_case(parameters):
    @lru_cache(maxsize=None)
    def run(name):
        return simulate(parameters, experiment_by_name(name))
    return run
