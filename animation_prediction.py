import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

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


def format_tcpa_sec(tcpa_sec: float) -> str:
    if np.isfinite(tcpa_sec):
        return f"{tcpa_sec:.0f} с"

    return "inf"


def phase_to_text(phase: str) -> str:
    names = {
        "NORMAL": "движение без маневра",
        "AVOIDING": "уклонение",
        "RETURN_TO_TRACK": "возврат на линию пути",
        "ON_ORIGINAL_TRACK": "движение после возврата",
        "NO_SAFE_MANEUVER": "без найденного безопасного маневра"
    }

    return names.get(phase, phase)


def build_animation_frame_indices(
    snapshots_count: int,
    frame_step: int
) -> list[int]:
    frame_step = max(1, int(frame_step))

    frame_indices = list(range(0, snapshots_count, frame_step))

    if frame_indices[-1] != snapshots_count - 1:
        frame_indices.append(snapshots_count - 1)

    return frame_indices


def get_trace_start_index(
    time_values_sec: np.ndarray,
    current_index: int,
    tail_length_sec: float | None
) -> int:
    if tail_length_sec is None:
        return 0

    current_time_sec = time_values_sec[current_index]
    min_time_sec = current_time_sec - tail_length_sec

    return int(np.searchsorted(time_values_sec, min_time_sec, side="left"))


def animate_encounter_prediction(
    snapshots: list[EncounterSnapshot],
    own_initial_state: ShipState,
    target_initial_state: ShipState,
    safe_distance_nm: float,
    frame_step: int = 5,
    interval_ms: int = 40,
    tail_length_sec: float | None = None,
    output_animation_path: str | None = None
) -> FuncAnimation:
    if not snapshots:
        raise ValueError("Список snapshots не должен быть пустым.")

    own_x = np.array([s.own_x_nm for s in snapshots], dtype=float)
    own_y = np.array([s.own_y_nm for s in snapshots], dtype=float)

    target_x = np.array([s.target_x_nm for s in snapshots], dtype=float)
    target_y = np.array([s.target_y_nm for s in snapshots], dtype=float)

    time_values_sec = np.array([s.time_sec for s in snapshots], dtype=float)

    frame_indices = build_animation_frame_indices(
        snapshots_count=len(snapshots),
        frame_step=frame_step
    )

    own_initial_direction = course_to_unit_vector(own_initial_state.course_deg)

    all_points_x = np.concatenate([own_x, target_x])
    all_points_y = np.concatenate([own_y, target_y])

    span = max(
        all_points_x.max() - all_points_x.min(),
        all_points_y.max() - all_points_y.min(),
        1.0
    )

    route_extension = span * 0.35 + 1.0

    own_route_start = (
        np.array([own_initial_state.x_nm, own_initial_state.y_nm], dtype=float)
        - own_initial_direction * route_extension
    )

    own_route_end = (
        np.array([own_initial_state.x_nm, own_initial_state.y_nm], dtype=float)
        + own_initial_direction * (span + route_extension)
    )

    fig, ax = plt.subplots(figsize=(11, 11))

    ax.plot(
        [own_route_start[0], own_route_end[0]],
        [own_route_start[1], own_route_end[1]],
        linestyle="-",
        linewidth=1.1,
        label="Исходная линия пути собственного судна"
    )

    ax.plot(
        own_x,
        own_y,
        linestyle="--",
        linewidth=1.0,
        alpha=0.35,
        label="Полная прогнозная траектория собственного судна"
    )

    ax.plot(
        target_x,
        target_y,
        linestyle="--",
        linewidth=1.0,
        alpha=0.35,
        label="Полная прогнозная траектория судна-цели"
    )

    ax.scatter(
        own_x[0],
        own_y[0],
        s=170,
        marker="o",
        facecolors="none",
        linewidths=2.0,
        label="Старт собственного судна"
    )

    ax.scatter(
        target_x[0],
        target_y[0],
        s=170,
        marker="s",
        facecolors="none",
        linewidths=2.0,
        label="Старт судна-цели"
    )

    maneuver_start = get_first_snapshot_by_phase(snapshots, "AVOIDING")
    return_start = get_first_snapshot_by_phase(snapshots, "RETURN_TO_TRACK")
    rejoin_point = get_first_snapshot_by_phase(snapshots, "ON_ORIGINAL_TRACK")

    if maneuver_start is not None:
        ax.scatter(
            maneuver_start.own_x_nm,
            maneuver_start.own_y_nm,
            s=120,
            marker="D",
            label=f"Начало маневра: t={maneuver_start.time_sec:.0f} с"
        )

    if return_start is not None:
        ax.scatter(
            return_start.own_x_nm,
            return_start.own_y_nm,
            s=120,
            marker="P",
            label=f"Начало возврата: t={return_start.time_sec:.0f} с"
        )

    if rejoin_point is not None:
        ax.scatter(
            rejoin_point.own_x_nm,
            rejoin_point.own_y_nm,
            s=130,
            marker="^",
            label=f"Выход на линию пути: t={rejoin_point.time_sec:.0f} с"
        )

    intersection = line_intersection_by_courses(
        own_initial_state,
        target_initial_state
    )

    if intersection is not None:
        ax.scatter(
            intersection[0],
            intersection[1],
            s=90,
            marker="+",
            label="Геометрическое пересечение исходных линий пути"
        )

    own_trace_line, = ax.plot(
        [],
        [],
        linestyle="-",
        linewidth=2.5,
        label="Пройденный путь собственного судна"
    )

    target_trace_line, = ax.plot(
        [],
        [],
        linestyle="-",
        linewidth=2.0,
        label="Пройденный путь судна-цели"
    )

    own_point, = ax.plot(
        [],
        [],
        marker="o",
        markersize=9,
        linestyle="None",
        label="Собственное судно"
    )

    target_point, = ax.plot(
        [],
        [],
        marker="s",
        markersize=9,
        linestyle="None",
        label="Судно-цель"
    )

    distance_line, = ax.plot(
        [],
        [],
        linestyle=":",
        linewidth=1.4,
        label="Текущая дистанция"
    )

    safe_circle = plt.Circle(
        (target_x[0], target_y[0]),
        safe_distance_nm,
        fill=False,
        linestyle="--",
        linewidth=1.3,
        label="Безопасная зона вокруг цели"
    )

    ax.add_patch(safe_circle)

    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.85
        }
    )

    all_x = np.concatenate([
        own_x,
        target_x,
        np.array([own_route_start[0], own_route_end[0]])
    ])

    all_y = np.concatenate([
        own_y,
        target_y,
        np.array([own_route_start[1], own_route_end[1]])
    ])

    x_range = all_x.max() - all_x.min()
    y_range = all_y.max() - all_y.min()

    margin = max(x_range, y_range, 0.5) * 0.12 + safe_distance_nm

    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_xlabel("X, морские мили")
    ax.set_ylabel("Y, морские мили")
    ax.set_title("Анимация движения судов при прогнозном проигрывании маневра")
    ax.legend(loc="best", fontsize=8)

    def update(display_frame_number: int):
        snapshot_index = frame_indices[display_frame_number]
        snapshot = snapshots[snapshot_index]

        trace_start_index = get_trace_start_index(
            time_values_sec=time_values_sec,
            current_index=snapshot_index,
            tail_length_sec=tail_length_sec
        )

        own_trace_line.set_data(
            own_x[trace_start_index:snapshot_index + 1],
            own_y[trace_start_index:snapshot_index + 1]
        )

        target_trace_line.set_data(
            target_x[trace_start_index:snapshot_index + 1],
            target_y[trace_start_index:snapshot_index + 1]
        )

        own_point.set_data(
            [snapshot.own_x_nm],
            [snapshot.own_y_nm]
        )

        target_point.set_data(
            [snapshot.target_x_nm],
            [snapshot.target_y_nm]
        )

        distance_line.set_data(
            [snapshot.own_x_nm, snapshot.target_x_nm],
            [snapshot.own_y_nm, snapshot.target_y_nm]
        )

        safe_circle.center = (
            snapshot.target_x_nm,
            snapshot.target_y_nm
        )

        info_text.set_text(
            f"t = {snapshot.time_sec:.0f} с\n"
            f"Фаза: {phase_to_text(snapshot.own_phase)}\n"
            f"D = {snapshot.distance_nm:.3f} м.м.\n"
            f"DCPA = {snapshot.dcpa_nm:.3f} м.м.\n"
            f"TCPA = {format_tcpa_sec(snapshot.tcpa_sec)}\n"
            f"Курс СО = {snapshot.own_course_deg:.1f}°\n"
            f"Скорость СО = {snapshot.own_speed_kn:.1f} уз.\n"
            f"Курс цели = {snapshot.target_course_deg:.1f}°\n"
            f"Скорость цели = {snapshot.target_speed_kn:.1f} уз."
        )

        return (
            own_trace_line,
            target_trace_line,
            own_point,
            target_point,
            distance_line,
            safe_circle,
            info_text
        )

    animation = FuncAnimation(
        fig=fig,
        func=update,
        frames=len(frame_indices),
        interval=interval_ms,
        blit=False,
        repeat=False
    )

    plt.tight_layout()

    if output_animation_path is not None:
        fps = max(1, int(round(1000 / interval_ms)))
        lower_path = output_animation_path.lower()

        if lower_path.endswith(".gif"):
            writer = PillowWriter(fps=fps)
        elif lower_path.endswith(".mp4"):
            writer = FFMpegWriter(fps=fps)
        else:
            raise ValueError("Поддерживаются расширения .gif и .mp4.")

        animation.save(output_animation_path, writer=writer)

    plt.show()

    return animation