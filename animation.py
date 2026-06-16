import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_ship_motion(snapshots, fps: int, vector_length: float):
    x_values = np.array([s.x for s in snapshots])
    y_values = np.array([s.y for s in snapshots])

    fig, ax = plt.subplots(figsize=(9, 9))

    margin = 2.0
    ax.set_xlim(x_values.min() - margin, x_values.max() + margin)
    ax.set_ylim(y_values.min() - margin, y_values.max() + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Простейшая симуляция поворота судна с управляющим импульсом")

    trajectory_line, = ax.plot([], [], linewidth=2, label="Пройденная траектория")

    start_marker = ax.scatter(
        x_values[0],
        y_values[0],
        s=80,
        marker="o",
        label=f"Начало: ({x_values[0]:.2f}, {y_values[0]:.2f})"
    )

    end_marker = ax.scatter(
        x_values[-1],
        y_values[-1],
        s=80,
        marker="s",
        label=f"Конец: ({x_values[-1]:.2f}, {y_values[-1]:.2f})"
    )

    current_point, = ax.plot(
        [],
        [],
        marker="o",
        markersize=10,
        linestyle="None",
        label="Текущее положение"
    )

    direction_vector = ax.quiver(
        x_values[0],
        y_values[0],
        0,
        vector_length,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.006,
        label="Вектор движения"
    )

    dynamic_info, = ax.plot(
        [],
        [],
        linestyle="None",
        label="Текущие параметры"
    )

    ax.legend(loc="upper left")

    def update(frame):
        snapshot = snapshots[frame]

        trajectory_line.set_data(x_values[:frame + 1], y_values[:frame + 1])
        current_point.set_data([snapshot.x], [snapshot.y])

        course_rad = np.deg2rad(snapshot.course_deg)
        vx_dir = np.sin(course_rad) * vector_length
        vy_dir = np.cos(course_rad) * vector_length

        direction_vector.set_offsets([[snapshot.x, snapshot.y]])
        direction_vector.set_UVC(vx_dir, vy_dir)

        current_point.set_label(
            f"Текущее положение: ({snapshot.x:.2f}, {snapshot.y:.2f})"
        )

        direction_vector.set_label(
            f"Вектор движения: vx={snapshot.vx:.3f}, vy={snapshot.vy:.3f}"
        )

        dynamic_info.set_label(
            f"t={snapshot.time:.1f} c; "
            f"K={snapshot.course_deg:.2f}°; "
            f"ω={snapshot.yaw_rate_deg:.2f}°/c; "
            f"α={snapshot.turn_accel_deg:.2f}°/c²"
        )

        ax.legend(loc="upper left")

        return trajectory_line, current_point, direction_vector, dynamic_info

    interval_ms = 1000 / fps

    animation = FuncAnimation(
        fig,
        update,
        frames=len(snapshots),
        interval=interval_ms,
        blit=False,
        repeat=False
    )

    plt.show()

    return animation