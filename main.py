from config import (
    DT_SEC,
    PREDICTION_HORIZON_SEC,
    OWN_INITIAL_X_NM,
    OWN_INITIAL_Y_NM,
    OWN_INITIAL_COURSE_DEG,
    OWN_INITIAL_SPEED_KN,
    TARGET_INITIAL_X_NM,
    TARGET_INITIAL_Y_NM,
    TARGET_INITIAL_COURSE_DEG,
    TARGET_INITIAL_SPEED_KN,
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
    OUTPUT_IMAGE_PATH,
    VARIABLE_DESCRIPTIONS,
    ENABLE_STATIC_PLOT,
    ENABLE_ANIMATION,
    ANIMATION_FRAME_STEP,
    ANIMATION_INTERVAL_MS,
    ANIMATION_TAIL_LENGTH_SEC,
    OUTPUT_ANIMATION_PATH,
)

from ship_model import (
    ShipState,
    Environment,
    MotionLimits,
    OriginalRoute,
    OwnShipPhase,
    EncounterSnapshot,
    copy_ship_state,
    normalize_course_deg,
    cross_track_error_nm,
    intercept_course_to_original_route_deg,
    step_controlled_ship,
    step_uncontrolled_ship
)

from collision_metrics import (
    calculate_cpa_metrics,
    is_dangerous_cpa
)

from maneuver_planner import (
    PlannerConfig,
    ManeuverPlan,
    should_activate_maneuver,
    can_start_return,
    find_best_maneuver
)

from plot_prediction import plot_encounter_prediction
from animation_prediction import animate_encounter_prediction


def print_variable_descriptions() -> None:
    print("Переменные, которые можно изменять в config.py:\n")

    for name, description in VARIABLE_DESCRIPTIONS.items():
        print(f"{name}: {description}")


def make_encounter_snapshot(
    time_sec: float,
    own_state: ShipState,
    target_state: ShipState,
    own_phase: OwnShipPhase,
    target_own_course_deg: float,
    target_own_speed_kn: float,
    original_route: OriginalRoute
) -> EncounterSnapshot:
    metrics = calculate_cpa_metrics(own_state, target_state)

    return EncounterSnapshot(
        time_sec=time_sec,
        own_x_nm=own_state.x_nm,
        own_y_nm=own_state.y_nm,
        own_course_deg=normalize_course_deg(own_state.course_deg),
        own_speed_kn=own_state.speed_kn,
        target_x_nm=target_state.x_nm,
        target_y_nm=target_state.y_nm,
        target_course_deg=normalize_course_deg(target_state.course_deg),
        target_speed_kn=target_state.speed_kn,
        own_phase=own_phase.value,
        target_own_course_deg=normalize_course_deg(target_own_course_deg),
        target_own_speed_kn=target_own_speed_kn,
        distance_nm=metrics.distance_nm,
        dcpa_nm=metrics.dcpa_nm,
        tcpa_sec=metrics.tcpa_sec,
        cross_track_error_nm=cross_track_error_nm(own_state, original_route)
    )


def simulate_encounter(
    own_initial_state: ShipState,
    target_initial_state: ShipState,
    motion_limits: MotionLimits,
    environment: Environment,
    planner_config: PlannerConfig,
    prediction_horizon_sec: float,
    dt_sec: float
) -> tuple[list[EncounterSnapshot], ManeuverPlan | None]:
    own_state = copy_ship_state(own_initial_state)
    target_state = copy_ship_state(target_initial_state)

    original_route = OriginalRoute(
        x0_nm=own_initial_state.x_nm,
        y0_nm=own_initial_state.y_nm,
        course_deg=normalize_course_deg(own_initial_state.course_deg),
        speed_kn=own_initial_state.speed_kn
    )

    snapshots: list[EncounterSnapshot] = []

    active_plan: ManeuverPlan | None = None
    selected_plan: ManeuverPlan | None = None

    own_phase = OwnShipPhase.NORMAL
    time_sec = 0.0

    while time_sec <= prediction_horizon_sec + 1e-9:
        target_own_course_deg = original_route.course_deg
        target_own_speed_kn = original_route.speed_kn

        if own_phase == OwnShipPhase.NORMAL:
            if should_activate_maneuver(own_state, target_state, planner_config):
                active_plan = find_best_maneuver(
                    own_state=own_state,
                    target_state=target_state,
                    original_route=original_route,
                    motion_limits=motion_limits,
                    environment=environment,
                    planner_config=planner_config,
                    remaining_horizon_sec=prediction_horizon_sec - time_sec,
                    dt_sec=dt_sec
                )

                if active_plan is None:
                    own_phase = OwnShipPhase.NO_SAFE_MANEUVER
                else:
                    selected_plan = active_plan
                    own_phase = OwnShipPhase.AVOIDING

        if own_phase == OwnShipPhase.AVOIDING:
            if active_plan is not None:
                target_own_course_deg = active_plan.target_course_deg
                target_own_speed_kn = active_plan.target_speed_kn

            if can_start_return(own_state, target_state, planner_config):
                own_phase = OwnShipPhase.RETURN_TO_TRACK

        if own_phase == OwnShipPhase.RETURN_TO_TRACK:
            if abs(cross_track_error_nm(own_state, original_route)) <= planner_config.route_rejoin_tolerance_nm:
                own_phase = OwnShipPhase.ON_ORIGINAL_TRACK

        if own_phase == OwnShipPhase.RETURN_TO_TRACK:
            target_own_course_deg = intercept_course_to_original_route_deg(
                state=own_state,
                original_route=original_route,
                route_lookahead_nm=planner_config.route_lookahead_nm
            )
            target_own_speed_kn = original_route.speed_kn

        elif own_phase == OwnShipPhase.ON_ORIGINAL_TRACK:
            target_own_course_deg = original_route.course_deg
            target_own_speed_kn = original_route.speed_kn

        elif own_phase == OwnShipPhase.NO_SAFE_MANEUVER:
            target_own_course_deg = original_route.course_deg
            target_own_speed_kn = original_route.speed_kn

        snapshots.append(
            make_encounter_snapshot(
                time_sec=time_sec,
                own_state=own_state,
                target_state=target_state,
                own_phase=own_phase,
                target_own_course_deg=target_own_course_deg,
                target_own_speed_kn=target_own_speed_kn,
                original_route=original_route
            )
        )

        if time_sec >= prediction_horizon_sec:
            break

        step_controlled_ship(
            state=own_state,
            target_course_deg=target_own_course_deg,
            target_speed_kn=target_own_speed_kn,
            limits=motion_limits,
            environment=environment,
            dt_sec=dt_sec
        )

        step_uncontrolled_ship(target_state, environment, dt_sec)

        time_sec += dt_sec

    return snapshots, selected_plan


def get_first_snapshot_by_phase(
    snapshots: list[EncounterSnapshot],
    phase_name: str
) -> EncounterSnapshot | None:
    for snapshot in snapshots:
        if snapshot.own_phase == phase_name:
            return snapshot

    return None


def print_initial_risk(own_state: ShipState, target_state: ShipState) -> None:
    metrics = calculate_cpa_metrics(own_state, target_state)

    dangerous = is_dangerous_cpa(
        metrics=metrics,
        safe_distance_nm=SAFE_DISTANCE_NM,
        tcpa_limit_sec=TCPA_LIMIT_SEC
    )

    print("\nНачальная оценка сближения:")
    print(f"Текущая дистанция: {metrics.distance_nm:.3f} м.м.")
    print(f"DCPA: {metrics.dcpa_nm:.3f} м.м.")

    if metrics.tcpa_sec == float("inf"):
        print("TCPA: бесконечность, относительная скорость близка к нулю")
    else:
        print(f"TCPA: {metrics.tcpa_sec:.0f} с")

    print(f"Опасное сближение в пределах {TCPA_LIMIT_SEC:.0f} с: {'да' if dangerous else 'нет'}")


def print_prediction_summary(
    snapshots: list[EncounterSnapshot],
    selected_plan: ManeuverPlan | None
) -> None:
    end = snapshots[-1]

    maneuver_start = get_first_snapshot_by_phase(snapshots, "AVOIDING")
    return_start = get_first_snapshot_by_phase(snapshots, "RETURN_TO_TRACK")
    rejoin_point = get_first_snapshot_by_phase(snapshots, "ON_ORIGINAL_TRACK")

    min_distance_snapshot = min(snapshots, key=lambda s: s.distance_nm)

    print("\nРезультат моделирования:")
    print(f"Горизонт прогноза: {end.time_sec:.0f} с")
    print(
        f"Минимальная дистанция за сценарий: "
        f"{min_distance_snapshot.distance_nm:.3f} м.м. "
        f"при t={min_distance_snapshot.time_sec:.0f} с"
    )

    if selected_plan is None:
        if maneuver_start is None:
            print("Маневр не применялся: опасное сближение не потребовало активации маневра или безопасный вариант не найден.")
        else:
            print("Безопасный маневр не найден.")
    else:
        print("\nВыбранный маневр:")
        print(f"Изменение курса: {selected_plan.course_delta_deg:+.1f}°")
        print(f"Заданный курс уклонения: {selected_plan.target_course_deg:.1f}°")
        print(f"Изменение скорости: {selected_plan.speed_delta_kn:+.1f} уз.")
        print(f"Заданная скорость уклонения: {selected_plan.target_speed_kn:.1f} уз.")
        print(f"Стоимость маневра: {selected_plan.cost:.2f}")
        print(f"Оценочная минимальная дистанция по кандидату: {selected_plan.min_distance_nm:.3f} м.м.")

    if maneuver_start is not None:
        print("\nНачало маневра:")
        print(f"t={maneuver_start.time_sec:.0f} с")
        print(f"X={maneuver_start.own_x_nm:.3f} м.м., Y={maneuver_start.own_y_nm:.3f} м.м.")

    if return_start is not None:
        print("\nНачало возврата:")
        print(f"t={return_start.time_sec:.0f} с")
        print(f"X={return_start.own_x_nm:.3f} м.м., Y={return_start.own_y_nm:.3f} м.м.")

    if rejoin_point is not None:
        print("\nВыход на исходную линию пути:")
        print(f"t={rejoin_point.time_sec:.0f} с")
        print(f"X={rejoin_point.own_x_nm:.3f} м.м., Y={rejoin_point.own_y_nm:.3f} м.м.")
        print(f"Отклонение от линии пути: {rejoin_point.cross_track_error_nm:.3f} м.м.")

    print("\nКонечное состояние собственного судна:")
    print(f"X={end.own_x_nm:.3f} м.м., Y={end.own_y_nm:.3f} м.м.")
    print(f"Курс={end.own_course_deg:.2f}°, скорость={end.own_speed_kn:.2f} уз.")

    print("\nКонечное состояние судна-цели:")
    print(f"X={end.target_x_nm:.3f} м.м., Y={end.target_y_nm:.3f} м.м.")
    print(f"Курс={end.target_course_deg:.2f}°, скорость={end.target_speed_kn:.2f} уз.")


def main() -> None:
    print_variable_descriptions()

    own_initial_state = ShipState(
        x_nm=OWN_INITIAL_X_NM,
        y_nm=OWN_INITIAL_Y_NM,
        course_deg=OWN_INITIAL_COURSE_DEG,
        speed_kn=OWN_INITIAL_SPEED_KN
    )

    target_initial_state = ShipState(
        x_nm=TARGET_INITIAL_X_NM,
        y_nm=TARGET_INITIAL_Y_NM,
        course_deg=TARGET_INITIAL_COURSE_DEG,
        speed_kn=TARGET_INITIAL_SPEED_KN
    )

    motion_limits = MotionLimits(
        max_turn_rate_deg_per_sec=MAX_TURN_RATE_DEG_PER_SEC,
        course_response_gain=COURSE_RESPONSE_GAIN,
        max_accel_kn_per_sec=MAX_ACCEL_KN_PER_SEC
    )

    environment = Environment(
        current_x_kn=CURRENT_X_KN,
        current_y_kn=CURRENT_Y_KN
    )

    planner_config = PlannerConfig(
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
        route_rejoin_tolerance_nm=ROUTE_REJOIN_TOLERANCE_NM
    )

    print_initial_risk(own_initial_state, target_initial_state)

    snapshots, selected_plan = simulate_encounter(
        own_initial_state=own_initial_state,
        target_initial_state=target_initial_state,
        motion_limits=motion_limits,
        environment=environment,
        planner_config=planner_config,
        prediction_horizon_sec=PREDICTION_HORIZON_SEC,
        dt_sec=DT_SEC
    )

    print_prediction_summary(snapshots, selected_plan)

    if ENABLE_STATIC_PLOT:
        plot_encounter_prediction(
            snapshots=snapshots,
            own_initial_state=own_initial_state,
            target_initial_state=target_initial_state,
            safe_distance_nm=SAFE_DISTANCE_NM,
            output_image_path=OUTPUT_IMAGE_PATH
        )

    if ENABLE_ANIMATION:
        animate_encounter_prediction(
            snapshots=snapshots,
            own_initial_state=own_initial_state,
            target_initial_state=target_initial_state,
            safe_distance_nm=SAFE_DISTANCE_NM,
            frame_step=ANIMATION_FRAME_STEP,
            interval_ms=ANIMATION_INTERVAL_MS,
            tail_length_sec=ANIMATION_TAIL_LENGTH_SEC,
            output_animation_path=OUTPUT_ANIMATION_PATH
        )


if __name__ == "__main__":
    main()