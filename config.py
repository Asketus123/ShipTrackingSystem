DT_SEC = 1.0
PREDICTION_HORIZON_SEC = 900.0

INITIAL_X_NM = 0.0
INITIAL_Y_NM = 0.0
INITIAL_COURSE_DEG = 0.0
INITIAL_SPEED_KN = 12.0

MANEUVER_TARGET_COURSE_DEG = 28.0
MANEUVER_TARGET_SPEED_KN = 15.0

RETURN_START_TIME_SEC = 200.0

MAX_TURN_RATE_DEG_PER_SEC = 0.35
COURSE_RESPONSE_GAIN = 0.08

MAX_ACCEL_KN_PER_SEC = 0.02

CURRENT_X_KN = 0.0
CURRENT_Y_KN = 0.0

ROUTE_LOOKAHEAD_NM = 1.0
ROUTE_REJOIN_TOLERANCE_NM = 0.03

VECTOR_LENGTH_NM = 0.5
OUTPUT_IMAGE_PATH = "trial_maneuver_return_to_track.png"

VARIABLE_DESCRIPTIONS = {
    "DT_SEC": "Шаг численного расчета, секунды.",
    "PREDICTION_HORIZON_SEC": "Общий горизонт прогноза N, секунды.",
    "INITIAL_X_NM": "Начальная координата X, морские мили. Ось X направлена на восток.",
    "INITIAL_Y_NM": "Начальная координата Y, морские мили. Ось Y направлена на север.",
    "INITIAL_COURSE_DEG": "Начальный истинный курс судна, градусы. 0 - север, 90 - восток.",
    "INITIAL_SPEED_KN": "Начальная скорость судна, узлы.",
    "MANEUVER_TARGET_COURSE_DEG": "Курс, на который судно должно перейти при маневре уклонения.",
    "MANEUVER_TARGET_SPEED_KN": "Скорость, на которую судно должно перейти при маневре уклонения.",
    "RETURN_START_TIME_SEC": "Момент начала возврата на исходную линию пути, секунды от начала прогноза.",
    "MAX_TURN_RATE_DEG_PER_SEC": "Максимальная скорость изменения курса, градусы в секунду.",
    "COURSE_RESPONSE_GAIN": "Коэффициент реакции на ошибку курса. Чем больше значение, тем резче доворачивание.",
    "MAX_ACCEL_KN_PER_SEC": "Максимальное изменение скорости, узлы в секунду.",
    "CURRENT_X_KN": "Составляющая течения по оси X, узлы. Положительное значение - на восток.",
    "CURRENT_Y_KN": "Составляющая течения по оси Y, узлы. Положительное значение - на север.",
    "ROUTE_LOOKAHEAD_NM": "Упреждающая точка на исходной линии пути, к которой строится курс перехвата.",
    "ROUTE_REJOIN_TOLERANCE_NM": "Допустимое расстояние до исходной линии пути, при котором считается, что судно вернулось на линию.",
    "VECTOR_LENGTH_NM": "Длина стрелок направления движения на графике, морские мили.",
    "OUTPUT_IMAGE_PATH": "Имя файла для сохранения графика."
}