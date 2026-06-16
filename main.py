from config import (
    DT_SEC,
    PREDICTION_HORIZON_SEC,
    INITIAL_X_NM,
    INITIAL_Y_NM,
    INITIAL_COURSE_DEG,
    INITIAL_SPEED_KN,
    MANEUVER_TARGET_COURSE_DEG,
    MANEUVER_TARGET_SPEED_KN,
    RETURN_START_TIME_SEC,
    MAX_TURN_RATE_DEG_PER_SEC,
    COURSE_RESPONSE_GAIN,
    MAX_ACCEL_KN_PER_SEC,
    CURRENT_X_KN,
    CURRENT_Y_KN,
    ROUTE_LOOKAHEAD_NM,
    ROUTE_REJOIN_TOLERANCE_NM,
    VECTOR_LENGTH_NM,
    OUTPUT_IMAGE_PATH,
    VARIABLE_DESCRIPTIONS
)

from ship_model import (
    ShipState,
    Environment,
    ManeuverScenario,
    simulate_maneuver_with_return
)

from plot_prediction import plot_maneuver_with_return


def print_variable_descriptions() -> None:
    print("Переменные, которые можно изменять в config.py:\n")

    for name, description in VARIABLE_DESCRIPTIONS.items():
        print(f"{name}: {description}")


def get_first_snapshot_by_phase(snapshots, phase_name):
    for snapshot in snapshots:
        if snapshot.phase == phase_name:
            return snapshot
    return None


def print_prediction_summary(snapshots) -> None:
    start = snapshots[0]
    end = snapshots[-1]
    return_start = get_first_snapshot_by_phase(snapshots, "RETURN_TO_TRACK")
    rejoin_point = get_first_snapshot_by_phase(snapshots, "ON_ORIGINAL_TRACK")

    print("\nРезультат прогнозного проигрывания маневра:")
    print(f"Горизонт прогноза: {end.time_sec:.0f} с")

    print("\nНачальное состояние:")
    print(f"X={start.x_nm:.3f} м.м., Y={start.y_nm:.3f} м.м.")
    print(f"Курс={start.course_deg:.2f}°, скорость={start.speed_kn:.2f} уз.")

    if return_start is not None:
        print("\nНачало возврата:")
        print(f"t={return_start.time_sec:.0f} с")
        print(f"X={return_start.x_nm:.3f} м.м., Y={return_start.y_nm:.3f} м.м.")
        print(f"Курс={return_start.course_deg:.2f}°, скорость={return_start.speed_kn:.2f} уз.")
        print(f"Отклонение от исходной линии пути={return_start.cross_track_error_nm:.3f} м.м.")

    if rejoin_point is not None:
        print("\nВыход на исходную линию пути:")
        print(f"t={rejoin_point.time_sec:.0f} с")
        print(f"X={rejoin_point.x_nm:.3f} м.м., Y={rejoin_point.y_nm:.3f} м.м.")
        print(f"Курс={rejoin_point.course_deg:.2f}°, скорость={rejoin_point.speed_kn:.2f} уз.")
        print(f"Отклонение от исходной линии пути={rejoin_point.cross_track_error_nm:.3f} м.м.")
    else:
        print("\nЗа заданный горизонт прогноза судно не вышло на исходную линию пути.")

    print("\nКонечное состояние:")
    print(f"X={end.x_nm:.3f} м.м., Y={end.y_nm:.3f} м.м.")
    print(f"Курс={end.course_deg:.2f}°, скорость={end.speed_kn:.2f} уз.")
    print(f"Отклонение от исходной линии пути={end.cross_track_error_nm:.3f} м.м.")


def main() -> None:
    print_variable_descriptions()

    initial_state = ShipState(
        x_nm=INITIAL_X_NM,
        y_nm=INITIAL_Y_NM,
        course_deg=INITIAL_COURSE_DEG,
        speed_kn=INITIAL_SPEED_KN
    )

    scenario = ManeuverScenario(
        maneuver_target_course_deg=MANEUVER_TARGET_COURSE_DEG,
        maneuver_target_speed_kn=MANEUVER_TARGET_SPEED_KN,
        return_start_time_sec=RETURN_START_TIME_SEC,
        max_turn_rate_deg_per_sec=MAX_TURN_RATE_DEG_PER_SEC,
        course_response_gain=COURSE_RESPONSE_GAIN,
        max_accel_kn_per_sec=MAX_ACCEL_KN_PER_SEC,
        route_lookahead_nm=ROUTE_LOOKAHEAD_NM,
        route_rejoin_tolerance_nm=ROUTE_REJOIN_TOLERANCE_NM
    )

    environment = Environment(
        current_x_kn=CURRENT_X_KN,
        current_y_kn=CURRENT_Y_KN
    )

    snapshots = simulate_maneuver_with_return(
        initial_state=initial_state,
        scenario=scenario,
        environment=environment,
        prediction_horizon_sec=PREDICTION_HORIZON_SEC,
        dt_sec=DT_SEC
    )

    print_prediction_summary(snapshots)

    plot_maneuver_with_return(
        snapshots=snapshots,
        initial_course_deg=INITIAL_COURSE_DEG,
        vector_length_nm=VECTOR_LENGTH_NM,
        output_image_path=OUTPUT_IMAGE_PATH
    )


if __name__ == "__main__":
    main()