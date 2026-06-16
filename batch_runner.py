import csv
import math
from pathlib import Path

from config import (
    DT_SEC,
    PREDICTION_HORIZON_SEC,
    SAFE_DISTANCE_NM,
    TCPA_LIMIT_SEC,
    MANEUVER_ACTIVATION_DISTANCE_NM,
    COURSE_DELTA_MIN_DEG,
    COURSE_DELTA_MAX_DEG,
    COURSE_DELTA_STEP_DEG,
    SPEED_DELTA_MIN_KN,
    SPEED_DELTA_MAX_KN,
    SPEED_DELTA_STEP_KN,
    MIN_ALLOWED_SPEED_KN,
    COURSE_COST_WEIGHT,
    SPEED_COST_WEIGHT,
    MAX_TURN_RATE_DEG_PER_SEC,
    COURSE_RESPONSE_GAIN,
    MAX_ACCEL_KN_PER_SEC,
    RETURN_CLEARANCE_FACTOR,
    ROUTE_LOOKAHEAD_NM,
    ROUTE_REJOIN_TOLERANCE_NM,
    CURRENT_X_KN,
    CURRENT_Y_KN,
)

from ship_model import (
    ShipState,
    Environment,
    MotionLimits,
    EncounterSnapshot,
    copy_ship_state,
    step_controlled_ship,
    step_uncontrolled_ship,
)

from collision_metrics import (
    calculate_cpa_metrics,
    calculate_distance_nm,
    is_dangerous_cpa,
)

from maneuver_planner import (
    PlannerConfig,
    ManeuverPlan,
)

from main import simulate_encounter
from scenarios import Scenario, get_control_scenarios


RESULTS_CSV_PATH = "batch_results.csv"


def make_motion_limits() -> MotionLimits:
    return MotionLimits(
        max_turn_rate_deg_per_sec=MAX_TURN_RATE_DEG_PER_SEC,
        course_response_gain=COURSE_RESPONSE_GAIN,
        max_accel_kn_per_sec=MAX_ACCEL_KN_PER_SEC,
    )


def make_environment() -> Environment:
    return Environment(
        current_x_kn=CURRENT_X_KN,
        current_y_kn=CURRENT_Y_KN,
    )


def make_planner_config() -> PlannerConfig:
    return PlannerConfig(
        safe_distance_nm=SAFE_DISTANCE_NM,
        tcpa_limit_sec=TCPA_LIMIT_SEC,
        maneuver_activation_distance_nm=MANEUVER_ACTIVATION_DISTANCE_NM,
        course_delta_min_deg=COURSE_DELTA_MIN_DEG,
        course_delta_max_deg=COURSE_DELTA_MAX_DEG,
        course_delta_step_deg=COURSE_DELTA_STEP_DEG,
        speed_delta_min_kn=SPEED_DELTA_MIN_KN,
        speed_delta_max_kn=SPEED_DELTA_MAX_KN,
        speed_delta_step_kn=SPEED_DELTA_STEP_KN,
        min_allowed_speed_kn=MIN_ALLOWED_SPEED_KN,
        course_cost_weight=COURSE_COST_WEIGHT,
        speed_cost_weight=SPEED_COST_WEIGHT,
        return_clearance_factor=RETURN_CLEARANCE_FACTOR,
        route_lookahead_nm=ROUTE_LOOKAHEAD_NM,
        route_rejoin_tolerance_nm=ROUTE_REJOIN_TOLERANCE_NM,
    )


def simulate_without_maneuver(
    own_initial_state: ShipState,
    target_initial_state: ShipState,
    motion_limits: MotionLimits,
    environment: Environment,
    prediction_horizon_sec: float,
    dt_sec: float,
) -> tuple[float, float]:
    own_state = copy_ship_state(own_initial_state)
    target_state = copy_ship_state(target_initial_state)

    min_distance_nm = math.inf
    min_distance_time_sec = 0.0

    time_sec = 0.0

    while time_sec <= prediction_horizon_sec + 1e-9:
        distance_nm = calculate_distance_nm(own_state, target_state)

        if distance_nm < min_distance_nm:
            min_distance_nm = distance_nm
            min_distance_time_sec = time_sec

        if time_sec >= prediction_horizon_sec:
            break

        step_controlled_ship(
            state=own_state,
            target_course_deg=own_initial_state.course_deg,
            target_speed_kn=own_initial_state.speed_kn,
            limits=motion_limits,
            environment=environment,
            dt_sec=dt_sec,
        )

        step_uncontrolled_ship(
            state=target_state,
            environment=environment,
            dt_sec=dt_sec,
        )

        time_sec += dt_sec

    return min_distance_nm, min_distance_time_sec


def get_min_distance_snapshot(
    snapshots: list[EncounterSnapshot],
) -> EncounterSnapshot:
    return min(snapshots, key=lambda snapshot: snapshot.distance_nm)


def has_phase(
    snapshots: list[EncounterSnapshot],
    phase_name: str,
) -> bool:
    return any(snapshot.own_phase == phase_name for snapshot in snapshots)


def make_expected_action_label(expected_maneuver: bool | None) -> str:
    if expected_maneuver is True:
        return "MANEUVER"

    if expected_maneuver is False:
        return "NO_MANEUVER"

    return "LIMITATION"


def classify_result(
    scenario: Scenario,
    snapshots: list[EncounterSnapshot],
    selected_plan: ManeuverPlan | None,
    initial_distance_nm: float,
    algorithm_min_distance_nm: float,
) -> str:
    maneuver_applied = selected_plan is not None
    no_safe_maneuver = has_phase(snapshots, "NO_SAFE_MANEUVER")
    safe_distance_violation = algorithm_min_distance_nm < SAFE_DISTANCE_NM - 1e-9

    if initial_distance_nm < SAFE_DISTANCE_NM:
        return "LIMITATION_INITIAL_UNSAFE"

    if scenario.expected_maneuver is True:
        if no_safe_maneuver:
            return "FAIL_NO_SAFE_MANEUVER"

        if not maneuver_applied:
            return "FAIL_MISSED_DANGER"

        if safe_distance_violation:
            return "FAIL_SAFE_DISTANCE"

        return "PASS"

    if scenario.expected_maneuver is False:
        if maneuver_applied:
            return "FAIL_FALSE_MANEUVER"

        if safe_distance_violation:
            return "FAIL_SAFE_DISTANCE"

        return "PASS"

    if safe_distance_violation:
        return "LIMITATION_INITIAL_UNSAFE"

    return "PASS"


def optional_float(value: float | None) -> float | str:
    if value is None:
        return ""

    if isinstance(value, float) and not math.isfinite(value):
        return "inf"

    return value


def make_result_row(
    scenario: Scenario,
    snapshots: list[EncounterSnapshot],
    selected_plan: ManeuverPlan | None,
    baseline_min_distance_nm: float,
    baseline_min_distance_time_sec: float,
) -> dict[str, object]:
    initial_metrics = calculate_cpa_metrics(
        scenario.own_initial_state,
        scenario.target_initial_state,
    )

    initial_dangerous_cpa = is_dangerous_cpa(
        metrics=initial_metrics,
        safe_distance_nm=SAFE_DISTANCE_NM,
        tcpa_limit_sec=TCPA_LIMIT_SEC,
    )

    min_distance_snapshot = get_min_distance_snapshot(snapshots)
    end_snapshot = snapshots[-1]

    maneuver_applied = selected_plan is not None
    return_started = has_phase(snapshots, "RETURN_TO_TRACK")
    rejoined_original_track = has_phase(snapshots, "ON_ORIGINAL_TRACK")
    no_safe_maneuver = has_phase(snapshots, "NO_SAFE_MANEUVER")

    result_status = classify_result(
        scenario=scenario,
        snapshots=snapshots,
        selected_plan=selected_plan,
        initial_distance_nm=initial_metrics.distance_nm,
        algorithm_min_distance_nm=min_distance_snapshot.distance_nm,
    )

    safety_margin_nm = min_distance_snapshot.distance_nm - SAFE_DISTANCE_NM
    safety_gain_nm = min_distance_snapshot.distance_nm - baseline_min_distance_nm

    course_delta_deg = selected_plan.course_delta_deg if selected_plan is not None else None
    speed_delta_kn = selected_plan.speed_delta_kn if selected_plan is not None else None
    maneuver_cost = selected_plan.cost if selected_plan is not None else None
    maneuver_duration_sec = selected_plan.estimated_maneuver_duration_sec if selected_plan is not None else None

    return {
        "scenario_id": scenario.scenario_id,
        "scenario_name": scenario.name,
        "expected_action": make_expected_action_label(scenario.expected_maneuver),
        "expected_maneuver": scenario.expected_maneuver if scenario.expected_maneuver is not None else "",
        "actual_action": "MANEUVER" if maneuver_applied else "NO_MANEUVER",
        "result_status": result_status,
        "comment": scenario.comment,

        "own_initial_x_nm": scenario.own_initial_state.x_nm,
        "own_initial_y_nm": scenario.own_initial_state.y_nm,
        "own_initial_course_deg": scenario.own_initial_state.course_deg,
        "own_initial_speed_kn": scenario.own_initial_state.speed_kn,

        "target_initial_x_nm": scenario.target_initial_state.x_nm,
        "target_initial_y_nm": scenario.target_initial_state.y_nm,
        "target_initial_course_deg": scenario.target_initial_state.course_deg,
        "target_initial_speed_kn": scenario.target_initial_state.speed_kn,

        "initial_distance_nm": initial_metrics.distance_nm,
        "initial_dcpa_nm": initial_metrics.dcpa_nm,
        "initial_tcpa_sec": initial_metrics.tcpa_sec,
        "initial_dangerous_cpa": initial_dangerous_cpa,

        "maneuver_applied": maneuver_applied,
        "no_safe_maneuver": no_safe_maneuver,
        "course_delta_deg": optional_float(course_delta_deg),
        "speed_delta_kn": optional_float(speed_delta_kn),
        "maneuver_cost": optional_float(maneuver_cost),
        "maneuver_duration_sec": optional_float(maneuver_duration_sec),

        "baseline_min_distance_nm": baseline_min_distance_nm,
        "baseline_min_distance_time_sec": baseline_min_distance_time_sec,

        "algorithm_min_distance_nm": min_distance_snapshot.distance_nm,
        "algorithm_min_distance_time_sec": min_distance_snapshot.time_sec,
        "safe_distance_nm": SAFE_DISTANCE_NM,
        "safety_margin_nm": safety_margin_nm,
        "safety_gain_nm": safety_gain_nm,
        "safe_distance_violation": min_distance_snapshot.distance_nm < SAFE_DISTANCE_NM - 1e-9,

        "return_started": return_started,
        "rejoined_original_track": rejoined_original_track,
        "final_cross_track_error_nm": end_snapshot.cross_track_error_nm,

        "final_own_x_nm": end_snapshot.own_x_nm,
        "final_own_y_nm": end_snapshot.own_y_nm,
        "final_own_course_deg": end_snapshot.own_course_deg,
        "final_own_speed_kn": end_snapshot.own_speed_kn,

        "final_target_x_nm": end_snapshot.target_x_nm,
        "final_target_y_nm": end_snapshot.target_y_nm,
        "final_target_course_deg": end_snapshot.target_course_deg,
        "final_target_speed_kn": end_snapshot.target_speed_kn,
    }


def run_scenario(
    scenario: Scenario,
    motion_limits: MotionLimits,
    environment: Environment,
    planner_config: PlannerConfig,
) -> dict[str, object]:
    baseline_min_distance_nm, baseline_min_distance_time_sec = simulate_without_maneuver(
        own_initial_state=scenario.own_initial_state,
        target_initial_state=scenario.target_initial_state,
        motion_limits=motion_limits,
        environment=environment,
        prediction_horizon_sec=PREDICTION_HORIZON_SEC,
        dt_sec=DT_SEC,
    )

    snapshots, selected_plan = simulate_encounter(
        own_initial_state=scenario.own_initial_state,
        target_initial_state=scenario.target_initial_state,
        motion_limits=motion_limits,
        environment=environment,
        planner_config=planner_config,
        prediction_horizon_sec=PREDICTION_HORIZON_SEC,
        dt_sec=DT_SEC,
    )

    return make_result_row(
        scenario=scenario,
        snapshots=snapshots,
        selected_plan=selected_plan,
        baseline_min_distance_nm=baseline_min_distance_nm,
        baseline_min_distance_time_sec=baseline_min_distance_time_sec,
    )


def write_results_csv(
    rows: list[dict[str, object]],
    output_path: str,
) -> None:
    if not rows:
        raise ValueError("Нет строк для записи в CSV.")

    fieldnames = list(rows[0].keys())

    with open(output_path, "w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def mean_or_none(values: list[float]) -> float | None:
    finite_values = [value for value in values if math.isfinite(value)]

    if not finite_values:
        return None

    return sum(finite_values) / len(finite_values)


def print_optional_float(
    label: str,
    value: float | None,
    precision: int = 3,
) -> None:
    if value is None:
        print(f"{label}: нет данных")
    else:
        print(f"{label}: {value:.{precision}f}")


def print_summary(rows: list[dict[str, object]]) -> None:
    total_count = len(rows)

    expected_maneuver_count = sum(row["expected_maneuver"] is True for row in rows)
    expected_no_maneuver_count = sum(row["expected_maneuver"] is False for row in rows)
    limitation_count = sum(row["expected_action"] == "LIMITATION" for row in rows)

    pass_count = sum(row["result_status"] == "PASS" for row in rows)
    false_maneuver_count = sum(row["result_status"] == "FAIL_FALSE_MANEUVER" for row in rows)
    missed_danger_count = sum(row["result_status"] == "FAIL_MISSED_DANGER" for row in rows)
    safe_distance_fail_count = sum(row["result_status"] == "FAIL_SAFE_DISTANCE" for row in rows)
    no_safe_maneuver_count = sum(row["result_status"] == "FAIL_NO_SAFE_MANEUVER" for row in rows)
    initial_unsafe_count = sum(row["result_status"] == "LIMITATION_INITIAL_UNSAFE" for row in rows)

    safety_margins = [float(row["safety_margin_nm"]) for row in rows]
    safety_gains = [float(row["safety_gain_nm"]) for row in rows]
    maneuver_costs = [
        float(row["maneuver_cost"])
        for row in rows
        if row["maneuver_cost"] != "" and str(row["maneuver_cost"]).lower() != "inf"
    ]

    maneuver_applied_count = sum(row["maneuver_applied"] is True for row in rows)
    return_started_count = sum(row["return_started"] is True for row in rows)
    rejoined_count = sum(row["rejoined_original_track"] is True for row in rows)

    tp = sum(
        row["expected_maneuver"] is True and row["maneuver_applied"] is True
        for row in rows
    )
    fn = sum(
        row["expected_maneuver"] is True and row["maneuver_applied"] is False
        for row in rows
    )
    fp = sum(
        row["expected_maneuver"] is False and row["maneuver_applied"] is True
        for row in rows
    )
    tn = sum(
        row["expected_maneuver"] is False and row["maneuver_applied"] is False
        for row in rows
    )

    print("\nСводка batch-проверки:")
    print(f"Всего сценариев: {total_count}")
    print(f"Сценариев с ожидаемым маневром: {expected_maneuver_count}")
    print(f"Сценариев без ожидаемого маневра: {expected_no_maneuver_count}")
    print(f"Нештатных сценариев: {limitation_count}")

    print("\nСтатусы:")
    print(f"PASS: {pass_count}")
    print(f"FAIL_FALSE_MANEUVER: {false_maneuver_count}")
    print(f"FAIL_MISSED_DANGER: {missed_danger_count}")
    print(f"FAIL_SAFE_DISTANCE: {safe_distance_fail_count}")
    print(f"FAIL_NO_SAFE_MANEUVER: {no_safe_maneuver_count}")
    print(f"LIMITATION_INITIAL_UNSAFE: {initial_unsafe_count}")

    print("\nМатрица срабатывания:")
    print(f"TP - маневр ожидался и применен: {tp}")
    print(f"FN - маневр ожидался, но не применен: {fn}")
    print(f"FP - маневр не ожидался, но применен: {fp}")
    print(f"TN - маневр не ожидался и не применен: {tn}")

    print("\nПоказатели безопасности:")
    print_optional_float("Средний запас безопасности, м.м.", mean_or_none(safety_margins))
    print_optional_float("Минимальный запас безопасности, м.м.", min(safety_margins))
    print_optional_float("Среднее улучшение относительно движения без маневра, м.м.", mean_or_none(safety_gains))
    print_optional_float("Максимальное улучшение относительно движения без маневра, м.м.", max(safety_gains))

    print("\nПоказатели маневрирования:")
    print(f"Количество сценариев с примененным маневром: {maneuver_applied_count}")
    print(f"Количество сценариев с начатым возвратом: {return_started_count}")
    print(f"Количество сценариев с выходом на исходную линию пути: {rejoined_count}")
    print_optional_float("Средняя стоимость маневра", mean_or_none(maneuver_costs), precision=2)


def main() -> None:
    scenarios = get_control_scenarios()

    motion_limits = make_motion_limits()
    environment = make_environment()
    planner_config = make_planner_config()

    rows = []

    for scenario in scenarios:
        row = run_scenario(
            scenario=scenario,
            motion_limits=motion_limits,
            environment=environment,
            planner_config=planner_config,
        )
        rows.append(row)

        print(
            f"{row['scenario_id']}: "
            f"{row['result_status']}, "
            f"min D alg = {float(row['algorithm_min_distance_nm']):.3f} м.м., "
            f"min D base = {float(row['baseline_min_distance_nm']):.3f} м.м., "
            f"action = {row['actual_action']}"
        )

    write_results_csv(rows, RESULTS_CSV_PATH)
    print_summary(rows)

    output_path = Path(RESULTS_CSV_PATH).resolve()
    print(f"\nCSV с результатами сохранен: {output_path}")


if __name__ == "__main__":
    main()