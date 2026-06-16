import numpy as np
from dataclasses import dataclass
from enum import Enum


class OwnShipPhase(str, Enum):
    NORMAL = "NORMAL"
    AVOIDING = "AVOIDING"
    RETURN_TO_TRACK = "RETURN_TO_TRACK"
    ON_ORIGINAL_TRACK = "ON_ORIGINAL_TRACK"
    NO_SAFE_MANEUVER = "NO_SAFE_MANEUVER"


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
class MotionLimits:
    max_turn_rate_deg_per_sec: float
    course_response_gain: float
    max_accel_kn_per_sec: float


@dataclass
class OriginalRoute:
    x0_nm: float
    y0_nm: float
    course_deg: float
    speed_kn: float


@dataclass
class EncounterSnapshot:
    time_sec: float
    own_x_nm: float
    own_y_nm: float
    own_course_deg: float
    own_speed_kn: float
    target_x_nm: float
    target_y_nm: float
    target_course_deg: float
    target_speed_kn: float
    own_phase: str
    target_own_course_deg: float
    target_own_speed_kn: float
    distance_nm: float
    dcpa_nm: float
    tcpa_sec: float
    cross_track_error_nm: float


def copy_ship_state(state: ShipState) -> ShipState:
    return ShipState(
        x_nm=state.x_nm,
        y_nm=state.y_nm,
        course_deg=state.course_deg,
        speed_kn=state.speed_kn
    )


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
        [state.x_nm - original_route.x0_nm, state.y_nm - original_route.y0_nm],
        dtype=float
    )
    return float(route_direction[0] * relative_position[1] - route_direction[1] * relative_position[0])


def projection_on_original_route_nm(state: ShipState, original_route: OriginalRoute) -> float:
    route_direction = course_to_unit_vector(original_route.course_deg)
    relative_position = np.array(
        [state.x_nm - original_route.x0_nm, state.y_nm - original_route.y0_nm],
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

    intercept_point = np.array([original_route.x0_nm, original_route.y0_nm], dtype=float)
    intercept_point = intercept_point + route_direction * (projection_nm + route_lookahead_nm)

    return bearing_from_point_to_point_deg(
        x_from_nm=state.x_nm,
        y_from_nm=state.y_nm,
        x_to_nm=float(intercept_point[0]),
        y_to_nm=float(intercept_point[1])
    )


def step_uncontrolled_ship(
    state: ShipState,
    environment: Environment,
    dt_sec: float
) -> None:
    vx_ship_kn, vy_ship_kn = course_to_velocity_components(
        speed_kn=state.speed_kn,
        course_deg=state.course_deg
    )

    vx_ground_kn = vx_ship_kn + environment.current_x_kn
    vy_ground_kn = vy_ship_kn + environment.current_y_kn

    dt_hours = dt_sec / 3600.0

    state.x_nm += vx_ground_kn * dt_hours
    state.y_nm += vy_ground_kn * dt_hours


def step_controlled_ship(
    state: ShipState,
    target_course_deg: float,
    target_speed_kn: float,
    limits: MotionLimits,
    environment: Environment,
    dt_sec: float
) -> tuple[float, float]:
    course_error_deg = signed_course_error_deg(
        target_course_deg=target_course_deg,
        current_course_deg=state.course_deg
    )

    desired_yaw_rate_deg_per_sec = limits.course_response_gain * course_error_deg

    yaw_rate_deg_per_sec = float(
        np.clip(
            desired_yaw_rate_deg_per_sec,
            -limits.max_turn_rate_deg_per_sec,
            limits.max_turn_rate_deg_per_sec
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
            -limits.max_accel_kn_per_sec * dt_sec,
            limits.max_accel_kn_per_sec * dt_sec
        )
    )

    acceleration_kn_per_sec = speed_delta_kn / dt_sec

    state.course_deg = normalize_course_deg(state.course_deg + course_delta_deg)
    state.speed_kn += speed_delta_kn

    step_uncontrolled_ship(state, environment, dt_sec)

    return yaw_rate_deg_per_sec, acceleration_kn_per_sec