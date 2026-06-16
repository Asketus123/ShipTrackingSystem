import numpy as np
from dataclasses import dataclass


@dataclass
class ShipState:
    x_nm: float
    y_nm: float
    course_deg: float
    speed_kn: float


@dataclass
class Environment:
    current_x_kn: float = 0.0
    current_y_kn: float = 0.0


@dataclass
class ManeuverScenario:
    maneuver_target_course_deg: float
    maneuver_target_speed_kn: float
    return_start_time_sec: float
    max_turn_rate_deg_per_sec: float
    course_response_gain: float
    max_accel_kn_per_sec: float
    route_lookahead_nm: float
    route_rejoin_tolerance_nm: float


@dataclass
class OriginalRoute:
    x0_nm: float
    y0_nm: float
    course_deg: float
    speed_kn: float


@dataclass
class ShipSnapshot:
    time_sec: float
    x_nm: float
    y_nm: float
    course_deg: float
    speed_kn: float
    target_course_deg: float
    target_speed_kn: float
    yaw_rate_deg_per_sec: float
    acceleration_kn_per_sec: float
    vx_ground_kn: float
    vy_ground_kn: float
    cross_track_error_nm: float
    phase: str


def normalize_course_deg(course_deg: float) -> float:
    return course_deg % 360.0


def signed_course_error_deg(target_course_deg: float, current_course_deg: float) -> float:
    return (target_course_deg - current_course_deg + 180.0) % 360.0 - 180.0


def course_to_unit_vector(course_deg: float) -> np.ndarray:
    course_rad = np.deg2rad(course_deg)
    return np.array([np.sin(course_rad), np.cos(course_rad)], dtype=float)


def course_to_velocity_components(speed_kn: float, course_deg: float) -> tuple[float, float]:
    direction = course_to_unit_vector(course_deg)
    return speed_kn * direction[0], speed_kn * direction[1]


def bearing_from_point_to_point_deg(
    x_from_nm: float,
    y_from_nm: float,
    x_to_nm: float,
    y_to_nm: float
) -> float:
    dx = x_to_nm - x_from_nm
    dy = y_to_nm - y_from_nm

    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return 0.0

    return normalize_course_deg(np.rad2deg(np.arctan2(dx, dy)))


def cross_track_error_nm(state: ShipState, original_route: OriginalRoute) -> float:
    route_direction = course_to_unit_vector(original_route.course_deg)
    relative_position = np.array(
        [
            state.x_nm - original_route.x0_nm,
            state.y_nm - original_route.y0_nm
        ],
        dtype=float
    )

    return route_direction[0] * relative_position[1] - route_direction[1] * relative_position[0]


def projection_on_original_route_nm(state: ShipState, original_route: OriginalRoute) -> float:
    route_direction = course_to_unit_vector(original_route.course_deg)
    relative_position = np.array(
        [
            state.x_nm - original_route.x0_nm,
            state.y_nm - original_route.y0_nm
        ],
        dtype=float
    )

    return float(np.dot(relative_position, route_direction))


def intercept_course_to_original_route_deg(
    state: ShipState,
    original_route: OriginalRoute,
    route_lookahead_nm: float
) -> float:
    route_direction = course_to_unit_vector(original_route.course_deg)
    projection_nm = projection_on_original_route_nm(state, original_route)

    intercept_point = np.array(
        [original_route.x0_nm, original_route.y0_nm],
        dtype=float
    ) + route_direction * (projection_nm + route_lookahead_nm)

    return bearing_from_point_to_point_deg(
        x_from_nm=state.x_nm,
        y_from_nm=state.y_nm,
        x_to_nm=intercept_point[0],
        y_to_nm=intercept_point[1]
    )


def make_snapshot(
    time_sec: float,
    state: ShipState,
    target_course_deg: float,
    target_speed_kn: float,
    yaw_rate_deg_per_sec: float,
    acceleration_kn_per_sec: float,
    environment: Environment,
    original_route: OriginalRoute,
    phase: str
) -> ShipSnapshot:
    vx_ship_kn, vy_ship_kn = course_to_velocity_components(
        speed_kn=state.speed_kn,
        course_deg=state.course_deg
    )

    vx_ground_kn = vx_ship_kn + environment.current_x_kn
    vy_ground_kn = vy_ship_kn + environment.current_y_kn

    return ShipSnapshot(
        time_sec=time_sec,
        x_nm=state.x_nm,
        y_nm=state.y_nm,
        course_deg=normalize_course_deg(state.course_deg),
        speed_kn=state.speed_kn,
        target_course_deg=normalize_course_deg(target_course_deg),
        target_speed_kn=target_speed_kn,
        yaw_rate_deg_per_sec=yaw_rate_deg_per_sec,
        acceleration_kn_per_sec=acceleration_kn_per_sec,
        vx_ground_kn=vx_ground_kn,
        vy_ground_kn=vy_ground_kn,
        cross_track_error_nm=cross_track_error_nm(state, original_route),
        phase=phase
    )


def apply_course_and_speed_control(
    state: ShipState,
    target_course_deg: float,
    target_speed_kn: float,
    scenario: ManeuverScenario,
    environment: Environment,
    dt_sec: float
) -> tuple[float, float]:
    course_error_deg = signed_course_error_deg(
        target_course_deg=target_course_deg,
        current_course_deg=state.course_deg
    )

    desired_yaw_rate_deg_per_sec = scenario.course_response_gain * course_error_deg

    yaw_rate_deg_per_sec = float(
        np.clip(
            desired_yaw_rate_deg_per_sec,
            -scenario.max_turn_rate_deg_per_sec,
            scenario.max_turn_rate_deg_per_sec
        )
    )

    course_delta_deg = yaw_rate_deg_per_sec * dt_sec

    if abs(course_delta_deg) > abs(course_error_deg):
        course_delta_deg = course_error_deg
        yaw_rate_deg_per_sec = course_delta_deg / dt_sec

    speed_error_kn = target_speed_kn - state.speed_kn

    speed_delta_kn = float(
        np.clip(
            speed_error_kn,
            -scenario.max_accel_kn_per_sec * dt_sec,
            scenario.max_accel_kn_per_sec * dt_sec
        )
    )

    acceleration_kn_per_sec = speed_delta_kn / dt_sec

    state.course_deg = normalize_course_deg(state.course_deg + course_delta_deg)
    state.speed_kn += speed_delta_kn

    vx_ship_kn, vy_ship_kn = course_to_velocity_components(
        speed_kn=state.speed_kn,
        course_deg=state.course_deg
    )

    vx_ground_kn = vx_ship_kn + environment.current_x_kn
    vy_ground_kn = vy_ship_kn + environment.current_y_kn

    dt_hours = dt_sec / 3600.0

    state.x_nm += vx_ground_kn * dt_hours
    state.y_nm += vy_ground_kn * dt_hours

    return yaw_rate_deg_per_sec, acceleration_kn_per_sec


def choose_targets_for_phase(
    time_sec: float,
    state: ShipState,
    scenario: ManeuverScenario,
    original_route: OriginalRoute,
    already_rejoined_route: bool
) -> tuple[float, float, str, bool]:
    if time_sec < scenario.return_start_time_sec:
        return (
            scenario.maneuver_target_course_deg,
            scenario.maneuver_target_speed_kn,
            "MANEUVER",
            already_rejoined_route
        )

    error_nm = abs(cross_track_error_nm(state, original_route))

    if already_rejoined_route or error_nm <= scenario.route_rejoin_tolerance_nm:
        return (
            original_route.course_deg,
            original_route.speed_kn,
            "ON_ORIGINAL_TRACK",
            True
        )

    intercept_course_deg = intercept_course_to_original_route_deg(
        state=state,
        original_route=original_route,
        route_lookahead_nm=scenario.route_lookahead_nm
    )

    return (
        intercept_course_deg,
        original_route.speed_kn,
        "RETURN_TO_TRACK",
        False
    )


def simulate_maneuver_with_return(
    initial_state: ShipState,
    scenario: ManeuverScenario,
    environment: Environment,
    prediction_horizon_sec: float,
    dt_sec: float
) -> list[ShipSnapshot]:
    state = ShipState(
        x_nm=initial_state.x_nm,
        y_nm=initial_state.y_nm,
        course_deg=normalize_course_deg(initial_state.course_deg),
        speed_kn=initial_state.speed_kn
    )

    original_route = OriginalRoute(
        x0_nm=initial_state.x_nm,
        y0_nm=initial_state.y_nm,
        course_deg=normalize_course_deg(initial_state.course_deg),
        speed_kn=initial_state.speed_kn
    )

    snapshots = []
    already_rejoined_route = False

    time_values = np.arange(0.0, prediction_horizon_sec + dt_sec, dt_sec)

    for time_sec in time_values:
        target_course_deg, target_speed_kn, phase, already_rejoined_route = choose_targets_for_phase(
            time_sec=float(time_sec),
            state=state,
            scenario=scenario,
            original_route=original_route,
            already_rejoined_route=already_rejoined_route
        )

        if time_sec == 0.0:
            yaw_rate_deg_per_sec = 0.0
            acceleration_kn_per_sec = 0.0
        else:
            yaw_rate_deg_per_sec, acceleration_kn_per_sec = apply_course_and_speed_control(
                state=state,
                target_course_deg=target_course_deg,
                target_speed_kn=target_speed_kn,
                scenario=scenario,
                environment=environment,
                dt_sec=dt_sec
            )

        snapshots.append(
            make_snapshot(
                time_sec=float(time_sec),
                state=state,
                target_course_deg=target_course_deg,
                target_speed_kn=target_speed_kn,
                yaw_rate_deg_per_sec=yaw_rate_deg_per_sec,
                acceleration_kn_per_sec=acceleration_kn_per_sec,
                environment=environment,
                original_route=original_route,
                phase=phase
            )
        )

    return snapshots