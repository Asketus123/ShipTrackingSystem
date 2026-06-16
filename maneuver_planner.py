import numpy as np
from dataclasses import dataclass

from ship_model import (
    ShipState,
    Environment,
    MotionLimits,
    OriginalRoute,
    OwnShipPhase,
    copy_ship_state,
    normalize_course_deg,
    cross_track_error_nm,
    intercept_course_to_original_route_deg,
    step_controlled_ship,
    step_uncontrolled_ship
)
from collision_metrics import calculate_cpa_metrics, calculate_distance_nm


@dataclass
class PlannerConfig:
    safe_distance_nm: float
    tcpa_limit_sec: float
    maneuver_activation_distance_nm: float
    course_delta_min_deg: int
    course_delta_max_deg: int
    course_delta_step_deg: int
    speed_delta_min_kn: int
    speed_delta_max_kn: int
    speed_delta_step_kn: int
    min_allowed_speed_kn: float
    course_cost_weight: float
    speed_cost_weight: float
    return_clearance_factor: float
    route_lookahead_nm: float
    route_rejoin_tolerance_nm: float


@dataclass
class ManeuverPlan:
    course_delta_deg: float
    speed_delta_kn: float
    target_course_deg: float
    target_speed_kn: float
    cost: float
    min_distance_nm: float
    min_distance_time_sec: float
    estimated_maneuver_duration_sec: float


def build_course_deltas(min_delta: int, max_delta: int, step: int) -> list[int]:
    max_abs = max(abs(min_delta), abs(max_delta))
    values = [0]

    for value in range(step, max_abs + step, step):
        if value <= max_delta:
            values.append(value)

        if -value >= min_delta:
            values.append(-value)

    return values


def build_speed_deltas(min_delta: int, max_delta: int, step: int) -> list[int]:
    max_abs = max(abs(min_delta), abs(max_delta))
    values = [0]

    for value in range(step, max_abs + step, step):
        if -value >= min_delta:
            values.append(-value)

        if value <= max_delta:
            values.append(value)

    return values


def maneuver_cost(
    course_delta_deg: float,
    speed_delta_kn: float,
    planner_config: PlannerConfig
) -> float:
    return (
        planner_config.course_cost_weight * abs(course_delta_deg)
        + planner_config.speed_cost_weight * abs(speed_delta_kn)
    )


def should_activate_maneuver(
    own_state: ShipState,
    target_state: ShipState,
    planner_config: PlannerConfig
) -> bool:
    metrics = calculate_cpa_metrics(own_state, target_state)
    current_distance = calculate_distance_nm(own_state, target_state)

    return (
        current_distance <= planner_config.maneuver_activation_distance_nm
        and metrics.dcpa_nm < planner_config.safe_distance_nm
        and 0.0 < metrics.tcpa_sec < planner_config.tcpa_limit_sec
    )


def can_start_return(
    own_state: ShipState,
    target_state: ShipState,
    planner_config: PlannerConfig
) -> bool:
    metrics = calculate_cpa_metrics(own_state, target_state)
    current_distance = calculate_distance_nm(own_state, target_state)

    return (
        current_distance > planner_config.safe_distance_nm * planner_config.return_clearance_factor
        and metrics.tcpa_sec < 0.0
    )


def simulate_candidate_until_return(
    own_state: ShipState,
    target_state: ShipState,
    original_route: OriginalRoute,
    candidate_target_course_deg: float,
    candidate_target_speed_kn: float,
    motion_limits: MotionLimits,
    environment: Environment,
    planner_config: PlannerConfig,
    remaining_horizon_sec: float,
    dt_sec: float
) -> tuple[bool, float, float, float]:
    own = copy_ship_state(own_state)
    target = copy_ship_state(target_state)

    phase = OwnShipPhase.AVOIDING

    min_distance_nm = float("inf")
    min_distance_time_sec = 0.0
    rejoin_time_sec = remaining_horizon_sec

    time_values = np.arange(0.0, remaining_horizon_sec + dt_sec, dt_sec)

    for local_time_sec in time_values:
        distance_nm = calculate_distance_nm(own, target)

        if distance_nm < min_distance_nm:
            min_distance_nm = distance_nm
            min_distance_time_sec = float(local_time_sec)

        if distance_nm < planner_config.safe_distance_nm:
            return False, min_distance_nm, min_distance_time_sec, rejoin_time_sec

        if phase == OwnShipPhase.AVOIDING and can_start_return(own, target, planner_config):
            phase = OwnShipPhase.RETURN_TO_TRACK

        if phase == OwnShipPhase.RETURN_TO_TRACK:
            if abs(cross_track_error_nm(own, original_route)) <= planner_config.route_rejoin_tolerance_nm:
                phase = OwnShipPhase.ON_ORIGINAL_TRACK
                rejoin_time_sec = float(local_time_sec)

        if phase == OwnShipPhase.AVOIDING:
            target_own_course_deg = candidate_target_course_deg
            target_own_speed_kn = candidate_target_speed_kn
        elif phase == OwnShipPhase.RETURN_TO_TRACK:
            target_own_course_deg = intercept_course_to_original_route_deg(
                state=own,
                original_route=original_route,
                route_lookahead_nm=planner_config.route_lookahead_nm
            )
            target_own_speed_kn = original_route.speed_kn
        else:
            target_own_course_deg = original_route.course_deg
            target_own_speed_kn = original_route.speed_kn

        step_controlled_ship(
            state=own,
            target_course_deg=target_own_course_deg,
            target_speed_kn=target_own_speed_kn,
            limits=motion_limits,
            environment=environment,
            dt_sec=dt_sec
        )

        step_uncontrolled_ship(target, environment, dt_sec)

    return True, min_distance_nm, min_distance_time_sec, rejoin_time_sec


def sort_key_for_plan(plan: ManeuverPlan) -> tuple[float, float, int, float, float]:
    right_turn_priority = 0 if plan.course_delta_deg >= 0 else 1

    return (
        plan.cost,
        plan.estimated_maneuver_duration_sec,
        right_turn_priority,
        abs(plan.course_delta_deg),
        abs(plan.speed_delta_kn)
    )


def find_best_maneuver(
    own_state: ShipState,
    target_state: ShipState,
    original_route: OriginalRoute,
    motion_limits: MotionLimits,
    environment: Environment,
    planner_config: PlannerConfig,
    remaining_horizon_sec: float,
    dt_sec: float
) -> ManeuverPlan | None:
    course_deltas = build_course_deltas(
        planner_config.course_delta_min_deg,
        planner_config.course_delta_max_deg,
        planner_config.course_delta_step_deg
    )

    speed_deltas = build_speed_deltas(
        planner_config.speed_delta_min_kn,
        planner_config.speed_delta_max_kn,
        planner_config.speed_delta_step_kn
    )

    candidates: list[ManeuverPlan] = []

    for course_delta_deg in course_deltas:
        for speed_delta_kn in speed_deltas:
            if course_delta_deg == 0 and speed_delta_kn == 0:
                continue

            candidate_target_course_deg = normalize_course_deg(own_state.course_deg + course_delta_deg)
            candidate_target_speed_kn = own_state.speed_kn + speed_delta_kn

            if candidate_target_speed_kn < planner_config.min_allowed_speed_kn:
                continue

            is_safe, min_distance_nm, min_distance_time_sec, maneuver_duration_sec = simulate_candidate_until_return(
                own_state=own_state,
                target_state=target_state,
                original_route=original_route,
                candidate_target_course_deg=candidate_target_course_deg,
                candidate_target_speed_kn=candidate_target_speed_kn,
                motion_limits=motion_limits,
                environment=environment,
                planner_config=planner_config,
                remaining_horizon_sec=remaining_horizon_sec,
                dt_sec=dt_sec
            )

            if not is_safe:
                continue

            candidates.append(
                ManeuverPlan(
                    course_delta_deg=float(course_delta_deg),
                    speed_delta_kn=float(speed_delta_kn),
                    target_course_deg=candidate_target_course_deg,
                    target_speed_kn=candidate_target_speed_kn,
                    cost=maneuver_cost(course_delta_deg, speed_delta_kn, planner_config),
                    min_distance_nm=min_distance_nm,
                    min_distance_time_sec=min_distance_time_sec,
                    estimated_maneuver_duration_sec=maneuver_duration_sec
                )
            )

    if not candidates:
        return None

    return sorted(candidates, key=sort_key_for_plan)[0]