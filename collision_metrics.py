import numpy as np
from dataclasses import dataclass

from ship_model import ShipState, course_to_velocity_components


@dataclass
class CollisionMetrics:
    distance_nm: float
    dcpa_nm: float
    tcpa_sec: float
    cpa_own_x_nm: float
    cpa_own_y_nm: float
    cpa_target_x_nm: float
    cpa_target_y_nm: float


def calculate_distance_nm(own: ShipState, target: ShipState) -> float:
    return float(np.hypot(target.x_nm - own.x_nm, target.y_nm - own.y_nm))


def calculate_cpa_metrics(own: ShipState, target: ShipState) -> CollisionMetrics:
    own_vx_kn, own_vy_kn = course_to_velocity_components(own.speed_kn, own.course_deg)
    target_vx_kn, target_vy_kn = course_to_velocity_components(target.speed_kn, target.course_deg)

    relative_position = np.array([target.x_nm - own.x_nm, target.y_nm - own.y_nm], dtype=float)
    relative_velocity = np.array([target_vx_kn - own_vx_kn, target_vy_kn - own_vy_kn], dtype=float)

    relative_speed_sq = float(np.dot(relative_velocity, relative_velocity))
    distance_nm = float(np.linalg.norm(relative_position))

    if relative_speed_sq < 1e-12:
        return CollisionMetrics(
            distance_nm=distance_nm,
            dcpa_nm=distance_nm,
            tcpa_sec=float("inf"),
            cpa_own_x_nm=own.x_nm,
            cpa_own_y_nm=own.y_nm,
            cpa_target_x_nm=target.x_nm,
            cpa_target_y_nm=target.y_nm
        )

    tcpa_hours = -float(np.dot(relative_position, relative_velocity)) / relative_speed_sq
    tcpa_sec = tcpa_hours * 3600.0

    own_cpa = np.array([own.x_nm, own.y_nm], dtype=float) + np.array([own_vx_kn, own_vy_kn]) * tcpa_hours
    target_cpa = np.array([target.x_nm, target.y_nm], dtype=float) + np.array([target_vx_kn, target_vy_kn]) * tcpa_hours

    dcpa_nm = float(np.linalg.norm(target_cpa - own_cpa))

    return CollisionMetrics(
        distance_nm=distance_nm,
        dcpa_nm=dcpa_nm,
        tcpa_sec=tcpa_sec,
        cpa_own_x_nm=float(own_cpa[0]),
        cpa_own_y_nm=float(own_cpa[1]),
        cpa_target_x_nm=float(target_cpa[0]),
        cpa_target_y_nm=float(target_cpa[1])
    )


def is_dangerous_cpa(
    metrics: CollisionMetrics,
    safe_distance_nm: float,
    tcpa_limit_sec: float
) -> bool:
    return (
        metrics.dcpa_nm < safe_distance_nm
        and 0.0 < metrics.tcpa_sec < tcpa_limit_sec
    )


def line_intersection_by_courses(
    ship_a: ShipState,
    ship_b: ShipState
) -> tuple[float, float] | None:
    ax, ay = course_to_velocity_components(1.0, ship_a.course_deg)
    bx, by = course_to_velocity_components(1.0, ship_b.course_deg)

    matrix = np.array([[ax, -bx], [ay, -by]], dtype=float)
    rhs = np.array([ship_b.x_nm - ship_a.x_nm, ship_b.y_nm - ship_a.y_nm], dtype=float)

    determinant = float(np.linalg.det(matrix))

    if abs(determinant) < 1e-10:
        return None

    t_a, _ = np.linalg.solve(matrix, rhs)
    point = np.array([ship_a.x_nm, ship_a.y_nm], dtype=float) + np.array([ax, ay]) * t_a

    return float(point[0]), float(point[1])