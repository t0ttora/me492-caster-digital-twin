"""Fixed demonstrations that call the published Python models."""


def torque_baseline():
    from backend import defaults, select_preset, simulate_config, validate

    config = select_preset(defaults("V1"), "preset", "misaligned_forward")
    params, duration = validate(config)
    rows, _, _ = simulate_config(config, params, duration)
    return [("Equal torque", rows, {})]


def compare_s_turn():
    from twin import run_case

    runs = []
    for timing in ("baseline", "smooth"):
        rows, metrics, _ = run_case("s_turn", timing, "reference_synthetic")
        runs.append((timing, rows, metrics))
    return runs
