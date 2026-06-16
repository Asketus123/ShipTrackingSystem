import csv
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from config import SAFE_DISTANCE_NM


RESULTS_CSV_PATH = "batch_results.csv"
OUTPUT_DIR = "batch_plots"


NUMERIC_COLUMNS = {
    "own_initial_x_nm",
    "own_initial_y_nm",
    "own_initial_course_deg",
    "own_initial_speed_kn",
    "target_initial_x_nm",
    "target_initial_y_nm",
    "target_initial_course_deg",
    "target_initial_speed_kn",
    "initial_distance_nm",
    "initial_dcpa_nm",
    "initial_tcpa_sec",
    "course_delta_deg",
    "speed_delta_kn",
    "maneuver_cost",
    "maneuver_duration_sec",
    "baseline_min_distance_nm",
    "baseline_min_distance_time_sec",
    "algorithm_min_distance_nm",
    "algorithm_min_distance_time_sec",
    "safe_distance_nm",
    "safety_margin_nm",
    "safety_gain_nm",
    "final_cross_track_error_nm",
    "final_own_x_nm",
    "final_own_y_nm",
    "final_own_course_deg",
    "final_own_speed_kn",
    "final_target_x_nm",
    "final_target_y_nm",
    "final_target_course_deg",
    "final_target_speed_kn",
}


BOOL_COLUMNS = {
    "expected_maneuver",
    "initial_dangerous_cpa",
    "maneuver_applied",
    "no_safe_maneuver",
    "safe_distance_violation",
    "return_started",
    "rejoined_original_track",
}


def parse_float(value: str) -> float:
    if value is None:
        return math.nan

    value = str(value).strip()

    if value == "":
        return math.nan

    if value.lower() == "inf":
        return math.inf

    return float(value)


def parse_optional_bool(value: str) -> bool | None:
    if value is None:
        return None

    value = str(value).strip().lower()

    if value == "":
        return None

    if value == "true":
        return True

    if value == "false":
        return False

    return None


def read_results_csv(input_path: str) -> list[dict[str, object]]:
    with open(input_path, "r", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        rows = list(reader)

    for row in rows:
        for column in NUMERIC_COLUMNS:
            if column in row:
                row[column] = parse_float(row[column])

        for column in BOOL_COLUMNS:
            if column in row:
                row[column] = parse_optional_bool(row[column])

    return rows


def scenario_ids(rows: list[dict[str, object]]) -> list[str]:
    return [str(row["scenario_id"]) for row in rows]


def ensure_output_dir(output_dir: str) -> Path:
    path = Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_figure(fig: plt.Figure, output_path: Path) -> None:
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def plot_min_distance_by_scenario(
    rows: list[dict[str, object]],
    output_dir: Path,
) -> None:
    labels = scenario_ids(rows)
    values = np.array([float(row["algorithm_min_distance_nm"]) for row in rows], dtype=float)

    x = np.arange(len(rows))

    fig, ax = plt.subplots(figsize=(14, 6))

    ax.bar(
        x,
        values,
        label="Минимальная дистанция при работе алгоритма"
    )

    ax.axhline(
        SAFE_DISTANCE_NM,
        linestyle="--",
        linewidth=1.5,
        label=f"Безопасная дистанция {SAFE_DISTANCE_NM:.2f} м.м."
    )

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45)
    ax.set_ylabel("Дистанция, морские мили")
    ax.set_title("Минимальная дистанция по контрольным сценариям")
    ax.grid(True, axis="y")
    ax.legend()

    save_figure(fig, output_dir / "01_min_distance_by_scenario.png")


def plot_min_distance_comparison(
    rows: list[dict[str, object]],
    output_dir: Path,
) -> None:
    labels = scenario_ids(rows)

    baseline_values = np.array([float(row["baseline_min_distance_nm"]) for row in rows], dtype=float)
    algorithm_values = np.array([float(row["algorithm_min_distance_nm"]) for row in rows], dtype=float)

    x = np.arange(len(rows))
    width = 0.38

    fig, ax = plt.subplots(figsize=(15, 6))

    ax.bar(
        x - width / 2,
        baseline_values,
        width,
        label="Без маневра"
    )

    ax.bar(
        x + width / 2,
        algorithm_values,
        width,
        label="С алгоритмом"
    )

    ax.axhline(
        SAFE_DISTANCE_NM,
        linestyle="--",
        linewidth=1.5,
        label=f"Безопасная дистанция {SAFE_DISTANCE_NM:.2f} м.м."
    )

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45)
    ax.set_ylabel("Минимальная дистанция, морские мили")
    ax.set_title("Сравнение минимальной дистанции с алгоритмом и без маневра")
    ax.grid(True, axis="y")
    ax.legend()

    save_figure(fig, output_dir / "02_min_distance_comparison.png")


def compute_confusion_matrix(
    rows: list[dict[str, object]],
) -> np.ndarray:
    tp = sum(
        row["expected_maneuver"] is True and row["maneuver_applied"] is True
        for row in rows
    )

    fn = sum(
        row["expected_maneuver"] is True and row["maneuver_applied"] is False
        for row in rows
    )

    fp = sum(
        row["expected_maneuver"] is False and row["maneuver_applied"] is True
        for row in rows
    )

    tn = sum(
        row["expected_maneuver"] is False and row["maneuver_applied"] is False
        for row in rows
    )

    return np.array(
        [
            [tp, fn],
            [fp, tn],
        ],
        dtype=int
    )


def plot_confusion_matrix(
    rows: list[dict[str, object]],
    output_dir: Path,
) -> None:
    matrix = compute_confusion_matrix(rows)

    fig, ax = plt.subplots(figsize=(7, 6))

    image = ax.imshow(matrix)

    ax.set_xticks([0, 1])
    ax.set_xticklabels(["Маневр применен", "Маневр не применен"])

    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Маневр ожидался", "Маневр не ожидался"])

    ax.set_title("Матрица корректности применения маневра")

    for row_index in range(matrix.shape[0]):
        for col_index in range(matrix.shape[1]):
            ax.text(
                col_index,
                row_index,
                str(matrix[row_index, col_index]),
                ha="center",
                va="center",
                fontsize=16
            )

    fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)

    save_figure(fig, output_dir / "03_maneuver_confusion_matrix.png")


def plot_cost_vs_safety_margin(
    rows: list[dict[str, object]],
    output_dir: Path,
) -> None:
    maneuver_rows = [
        row for row in rows
        if row["maneuver_applied"] is True
        and math.isfinite(float(row["maneuver_cost"]))
        and math.isfinite(float(row["safety_margin_nm"]))
    ]

    fig, ax = plt.subplots(figsize=(10, 6))

    if maneuver_rows:
        costs = np.array([float(row["maneuver_cost"]) for row in maneuver_rows], dtype=float)
        margins = np.array([float(row["safety_margin_nm"]) for row in maneuver_rows], dtype=float)

        ax.scatter(
            costs,
            margins,
            s=70,
            label="Сценарий с примененным маневром"
        )

        for row, cost, margin in zip(maneuver_rows, costs, margins):
            ax.annotate(
                str(row["scenario_id"]),
                (cost, margin),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8
            )
    else:
        ax.text(
            0.5,
            0.5,
            "Нет сценариев с примененным маневром",
            transform=ax.transAxes,
            ha="center",
            va="center"
        )

    ax.axhline(
        0.0,
        linestyle="--",
        linewidth=1.5,
        label="Граница безопасности"
    )

    ax.set_xlabel("Стоимость маневра")
    ax.set_ylabel("Запас безопасности, морские мили")
    ax.set_title("Стоимость маневра и достигнутый запас безопасности")
    ax.grid(True)
    ax.legend()

    save_figure(fig, output_dir / "04_cost_vs_safety_margin.png")


def plot_course_and_speed_deltas(
    rows: list[dict[str, object]],
    output_dir: Path,
) -> None:
    maneuver_rows = [
        row for row in rows
        if row["maneuver_applied"] is True
        and math.isfinite(float(row["course_delta_deg"]))
        and math.isfinite(float(row["speed_delta_kn"]))
    ]

    fig, ax_course = plt.subplots(figsize=(12, 6))

    if not maneuver_rows:
        ax_course.text(
            0.5,
            0.5,
            "Нет сценариев с примененным маневром",
            transform=ax_course.transAxes,
            ha="center",
            va="center"
        )
        ax_course.set_title("Изменение курса и скорости при выбранном маневре")
        save_figure(fig, output_dir / "05_course_and_speed_deltas.png")
        return

    labels = [str(row["scenario_id"]) for row in maneuver_rows]
    course_deltas = np.array([float(row["course_delta_deg"]) for row in maneuver_rows], dtype=float)
    speed_deltas = np.array([float(row["speed_delta_kn"]) for row in maneuver_rows], dtype=float)

    x = np.arange(len(maneuver_rows))
    width = 0.38

    course_bars = ax_course.bar(
        x - width / 2,
        course_deltas,
        width,
        label="Изменение курса, градусы"
    )

    ax_speed = ax_course.twinx()

    speed_bars = ax_speed.bar(
        x + width / 2,
        speed_deltas,
        width,
        label="Изменение скорости, узлы"
    )

    ax_course.axhline(0.0, linewidth=1.0)
    ax_speed.axhline(0.0, linewidth=1.0)

    ax_course.set_xticks(x)
    ax_course.set_xticklabels(labels, rotation=45)

    ax_course.set_ylabel("Изменение курса, градусы")
    ax_speed.set_ylabel("Изменение скорости, узлы")

    ax_course.set_title("Изменение курса и скорости при выбранном маневре")
    ax_course.grid(True, axis="y")

    handles = [course_bars, speed_bars]
    labels_for_legend = [handle.get_label() for handle in handles]
    ax_course.legend(handles, labels_for_legend, loc="best")

    save_figure(fig, output_dir / "05_course_and_speed_deltas.png")


def print_generated_files(output_dir: Path) -> None:
    print("\nПостроены графики:")

    for path in sorted(output_dir.glob("*.png")):
        print(path.resolve())


def main() -> None:
    rows = read_results_csv(RESULTS_CSV_PATH)
    output_dir = ensure_output_dir(OUTPUT_DIR)

    plot_min_distance_by_scenario(rows, output_dir)
    plot_min_distance_comparison(rows, output_dir)
    plot_confusion_matrix(rows, output_dir)
    plot_cost_vs_safety_margin(rows, output_dir)
    plot_course_and_speed_deltas(rows, output_dir)

    print_generated_files(output_dir)


if __name__ == "__main__":
    main()