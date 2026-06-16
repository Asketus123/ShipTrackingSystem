import numpy as np
import matplotlib.pyplot as plt

from ship_model import EncounterSnapshot, ShipState, course_to_unit_vector
from collision_metrics import line_intersection_by_courses


def get_first_snapshot_by_phase(
    snapshots: list[EncounterSnapshot],
    phase_name: str
) -> EncounterSnapshot | None:
    for snapshot in snapshots:
        if snapshot.own_phase == phase_name:
            return snapshot

    return None


def plot_encounter_prediction(
    snapshots: list[EncounterSnapshot],
    own_initial_state: ShipState,
    target_initial_state: ShipState,
    safe_distance_nm: float,
    output_image_path: str | None = None
) -> None:
    own_x = np.array([s.own_x_nm for s in snapshots])
    own_y = np.array([s.own_y_nm for s in snapshots])

    target_x = np.array([s.target_x_nm for s in snapshots])
    target_y = np.array([s.target_y_nm for s in snapshots])

    fig, ax = plt.subplots(figsize=(11, 11))

    start = snapshots[0]
    end = snapshots[-1]

    maneuver_start = get_first_snapshot_by_phase(snapshots, "AVOIDING")
    return_start = get_first_snapshot_by_phase(snapshots, "RETURN_TO_TRACK")
    rejoin_point = get_first_snapshot_by_phase(snapshots, "ON_ORIGINAL_TRACK")

    min_distance_snapshot = min(snapshots, key=lambda s: s.distance_nm)

    own_initial_direction = course_to_unit_vector(own_initial_state.course_deg)
    target_initial_direction = course_to_unit_vector(target_initial_state.course_deg)

    all_points_x = np.concatenate([own_x, target_x])
    all_points_y = np.concatenate([own_y, target_y])

    span = max(
        all_points_x.max() - all_points_x.min(),
        all_points_y.max() - all_points_y.min(),
        1.0
    )

    route_extension = span * 0.35 + 1.0

    own_route_start = np.array([own_initial_state.x_nm, own_initial_state.y_nm]) - own_initial_direction * route_extension
    own_route_end = np.array([own_initial_state.x_nm, own_initial_state.y_nm]) + own_initial_direction * (span + route_extension)

    target_route_start = np.array([target_initial_state.x_nm, target_initial_state.y_nm]) - target_initial_direction * route_extension
    target_route_end = np.array([target_initial_state.x_nm, target_initial_state.y_nm]) + target_initial_direction * (span + route_extension)

    ax.plot(
        [own_route_start[0], own_route_end[0]],
        [own_route_start[1], own_route_end[1]],
        linestyle="-",
        linewidth=1.1,
        label="Исходная линия пути собственного судна"
    )


    phases = [s.own_phase for s in snapshots]

    phase_styles = {
        "NORMAL": ("--", "Собственное судно до маневра"),
        "AVOIDING": ("-", "Маневр уклонения собственного судна"),
        "RETURN_TO_TRACK": (":", "Возврат на исходную линию пути"),
        "ON_ORIGINAL_TRACK": ("--", "Движение после возврата"),
        "NO_SAFE_MANEUVER": ("--", "Движение без найденного безопасного маневра")
    }

    for phase_name, style_data in phase_styles.items():
        indices = [i for i, phase in enumerate(phases) if phase == phase_name]

        if not indices:
            continue

        linestyle, label = style_data

        ax.plot(
            own_x[indices],
            own_y[indices],
            linestyle=linestyle,
            linewidth=2.4,
            label=label
        )

    ax.plot(
        target_x,
        target_y,
        linestyle="-",
        linewidth=2.0,
        label="Траектория судна-цели"
    )

    if maneuver_start is not None:
        ax.scatter(
            maneuver_start.own_x_nm,
            maneuver_start.own_y_nm,
            s=130,
            marker="D",
            label=f"Начало маневра: t={maneuver_start.time_sec:.0f} с"
        )

    if return_start is not None:
        ax.scatter(
            return_start.own_x_nm,
            return_start.own_y_nm,
            s=130,
            marker="P",
            label=f"Начало возврата: t={return_start.time_sec:.0f} с"
        )

    if rejoin_point is not None:
        ax.scatter(
            rejoin_point.own_x_nm,
            rejoin_point.own_y_nm,
            s=140,
            marker="^",
            label=f"Выход на линию пути: t={rejoin_point.time_sec:.0f} с"
        )
    ax.scatter(
        start.own_x_nm,
        start.own_y_nm,
        s=230,
        marker="o",
        facecolors="none",
        linewidths=2.2,
        zorder=10,
        label="Старт собственного судна"
    )

    ax.scatter(
        start.target_x_nm,
        start.target_y_nm,
        s=180,
        marker="s",
        facecolors="none",
        linewidths=2.2,
        zorder=10,
        label="Старт судна-цели"
    )
    ax.scatter(
        end.own_x_nm,
        end.own_y_nm,
        s=170,
        marker="o",
        facecolors="none",
        linewidths=2,
        label="Конец прогноза собственного судна"
    )

    ax.scatter(
        end.target_x_nm,
        end.target_y_nm,
        s=170,
        marker="s",
        facecolors="none",
        linewidths=2,
        label="Конец прогноза судна-цели"
    )

    ax.scatter(
        min_distance_snapshot.own_x_nm,
        min_distance_snapshot.own_y_nm,
        s=110,
        marker="x",
        label=f"Минимальная дистанция: {min_distance_snapshot.distance_nm:.3f} м.м."
    )

    safe_circle = plt.Circle(
        (min_distance_snapshot.target_x_nm, min_distance_snapshot.target_y_nm),
        safe_distance_nm,
        fill=False,
        linestyle="--",
        linewidth=1.4,
        label="Зона безопасной дистанции у цели"
    )

    ax.add_patch(safe_circle)

    intersection = line_intersection_by_courses(own_initial_state, target_initial_state)

    if intersection is not None:
        ax.scatter(
            intersection[0],
            intersection[1],
            s=90,
            marker="+",
            label="Геометрическое пересечение исходных линий пути"
        )

    all_x = np.concatenate([
        own_x,
        target_x,
        np.array([own_route_start[0], own_route_end[0], target_route_start[0], target_route_end[0]])
    ])

    all_y = np.concatenate([
        own_y,
        target_y,
        np.array([own_route_start[1], own_route_end[1], target_route_start[1], target_route_end[1]])
    ])

    x_range = all_x.max() - all_x.min()
    y_range = all_y.max() - all_y.min()

    margin = max(x_range, y_range, 0.5) * 0.12

    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_xlabel("X, морские мили")
    ax.set_ylabel("Y, морские мили")
    ax.set_title("Автоматический запуск маневра при опасном сближении")

    ax.legend(loc="best", fontsize=8)
    plt.tight_layout()

    if output_image_path is not None:
        plt.savefig(output_image_path, dpi=200)

    plt.show()