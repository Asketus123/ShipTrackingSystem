import numpy as np
import matplotlib.pyplot as plt

from ship_model import ShipSnapshot, course_to_unit_vector


def direction_vector(course_deg: float, length_nm: float) -> tuple[float, float]:
    direction = course_to_unit_vector(course_deg)
    return direction[0] * length_nm, direction[1] * length_nm


def get_first_snapshot_by_phase(
    snapshots: list[ShipSnapshot],
    phase_name: str
) -> ShipSnapshot | None:
    for snapshot in snapshots:
        if snapshot.phase == phase_name:
            return snapshot
    return None


def plot_maneuver_with_return(
    snapshots: list[ShipSnapshot],
    initial_course_deg: float,
    vector_length_nm: float,
    output_image_path: str | None = None
) -> None:
    x = np.array([s.x_nm for s in snapshots])
    y = np.array([s.y_nm for s in snapshots])

    fig, ax = plt.subplots(figsize=(10, 10))

    start = snapshots[0]
    end = snapshots[-1]
    return_start = get_first_snapshot_by_phase(snapshots, "RETURN_TO_TRACK")
    rejoin_point = get_first_snapshot_by_phase(snapshots, "ON_ORIGINAL_TRACK")

    initial_direction = course_to_unit_vector(initial_course_deg)

    route_projection = (
        (x - start.x_nm) * initial_direction[0]
        + (y - start.y_nm) * initial_direction[1]
    )

    min_projection = min(route_projection.min(), 0.0) - 0.5
    max_projection = max(route_projection.max(), 0.0) + 1.5

    route_start = np.array([start.x_nm, start.y_nm]) + initial_direction * min_projection
    route_end = np.array([start.x_nm, start.y_nm]) + initial_direction * max_projection

    ax.plot(
        [route_start[0], route_end[0]],
        [route_start[1], route_end[1]],
        linestyle="-",
        linewidth=1.2,
        label="Исходная линия пути"
    )

    phases = [snapshot.phase for snapshot in snapshots]

    phase_styles = {
        "MANEUVER": ("--", "Маневр уклонения"),
        "RETURN_TO_TRACK": (":", "Возврат к исходной линии пути"),
        "ON_ORIGINAL_TRACK": ("-.", "Движение после выхода на исходную линию")
    }

    for phase_name, style_data in phase_styles.items():
        linestyle, label = style_data
        indices = [i for i, phase in enumerate(phases) if phase == phase_name]

        if not indices:
            continue

        ax.plot(
            x[indices],
            y[indices],
            linestyle=linestyle,
            linewidth=2.4,
            label=label
        )

    ax.scatter(
        start.x_nm,
        start.y_nm,
        s=90,
        marker="o",
        label=(
            f"Начало: X={start.x_nm:.3f} м.м., "
            f"Y={start.y_nm:.3f} м.м."
        )
    )

    if return_start is not None:
        ax.scatter(
            return_start.x_nm,
            return_start.y_nm,
            s=120,
            marker="D",
            label=(
                f"Начало возврата: t={return_start.time_sec:.0f} с, "
                f"X={return_start.x_nm:.3f}, Y={return_start.y_nm:.3f}"
            )
        )

    if rejoin_point is not None:
        ax.scatter(
            rejoin_point.x_nm,
            rejoin_point.y_nm,
            s=130,
            marker="^",
            label=(
                f"Выход на линию пути: t={rejoin_point.time_sec:.0f} с, "
                f"X={rejoin_point.x_nm:.3f}, Y={rejoin_point.y_nm:.3f}"
            )
        )

    ax.scatter(
        end.x_nm,
        end.y_nm,
        s=170,
        marker="o",
        facecolors="none",
        linewidths=2,
        label=(
            f"Конец прогноза: t={end.time_sec:.0f} с, "
            f"X={end.x_nm:.3f}, Y={end.y_nm:.3f}"
        )
    )

    for snapshot, label_prefix in [
        (start, "Начальный вектор"),
        (end, "Конечный вектор")
    ]:
        dx, dy = direction_vector(snapshot.course_deg, vector_length_nm)

        ax.quiver(
            snapshot.x_nm,
            snapshot.y_nm,
            dx,
            dy,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.004,
            label=(
                f"{label_prefix}: курс={snapshot.course_deg:.1f}°, "
                f"скорость={snapshot.speed_kn:.1f} уз."
            )
        )

    all_x = np.concatenate([x, np.array([route_start[0], route_end[0]])])
    all_y = np.concatenate([y, np.array([route_start[1], route_end[1]])])

    x_range = all_x.max() - all_x.min()
    y_range = all_y.max() - all_y.min()
    margin = max(x_range, y_range, 0.5) * 0.15

    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_xlabel("X, морские мили")
    ax.set_ylabel("Y, морские мили")
    ax.set_title("Прогнозный маневр с возвратом на исходную линию пути")

    ax.legend(loc="best")
    plt.tight_layout()

    if output_image_path is not None:
        plt.savefig(output_image_path, dpi=200)

    plt.show()