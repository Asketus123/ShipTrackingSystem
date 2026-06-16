DT_SEC = 1.0
PREDICTION_HORIZON_SEC = 1800.0

OWN_INITIAL_X_NM = 0.0
OWN_INITIAL_Y_NM = -2.5
OWN_INITIAL_COURSE_DEG = 0.0
OWN_INITIAL_SPEED_KN = 15.0

TARGET_INITIAL_X_NM = -2.0
TARGET_INITIAL_Y_NM = 3.0
TARGET_INITIAL_COURSE_DEG = 135.0
TARGET_INITIAL_SPEED_KN = 9.0

SAFE_DISTANCE_NM = 0.5
TCPA_LIMIT_SEC = 900.0
MANEUVER_ACTIVATION_DISTANCE_NM = 3.0

COURSE_DELTA_MIN_DEG = -30
COURSE_DELTA_MAX_DEG = 30
COURSE_DELTA_STEP_DEG = 5

SPEED_DELTA_MIN_KN = -6
SPEED_DELTA_MAX_KN = 6
SPEED_DELTA_STEP_KN = 1
MIN_ALLOWED_SPEED_KN = 0.5

COURSE_COST_WEIGHT = 1.0
SPEED_COST_WEIGHT = 2.0

MAX_TURN_RATE_DEG_PER_SEC = 0.35
COURSE_RESPONSE_GAIN = 0.08
MAX_ACCEL_KN_PER_SEC = 0.02

RETURN_CLEARANCE_FACTOR = 1.5
ROUTE_LOOKAHEAD_NM = 1.0
ROUTE_REJOIN_TOLERANCE_NM = 0.03

CURRENT_X_KN = 0.0
CURRENT_Y_KN = 0.0
ENABLE_STATIC_PLOT = False
ENABLE_ANIMATION = True

ANIMATION_FRAME_STEP = 5
ANIMATION_INTERVAL_MS = 40
ANIMATION_TAIL_LENGTH_SEC = None
OUTPUT_ANIMATION_PATH = None

OUTPUT_IMAGE_PATH = "auto_maneuver_prediction.png"

VARIABLE_DESCRIPTIONS = {
    "DT_SEC": "Шаг численного расчета, секунды.",
    "PREDICTION_HORIZON_SEC": "Общий горизонт прогноза, секунды.",
    "OWN_INITIAL_X_NM": "Начальная координата X собственного судна, морские мили.",
    "OWN_INITIAL_Y_NM": "Начальная координата Y собственного судна, морские мили.",
    "OWN_INITIAL_COURSE_DEG": "Начальный курс собственного судна, градусы. 0 - север, 90 - восток.",
    "OWN_INITIAL_SPEED_KN": "Начальная скорость собственного судна, узлы.",
    "TARGET_INITIAL_X_NM": "Начальная координата X судна-цели, морские мили.",
    "TARGET_INITIAL_Y_NM": "Начальная координата Y судна-цели, морские мили.",
    "TARGET_INITIAL_COURSE_DEG": "Курс судна-цели, градусы. Цель не меняет курс.",
    "TARGET_INITIAL_SPEED_KN": "Скорость судна-цели, узлы. Цель не меняет скорость.",
    "SAFE_DISTANCE_NM": "Минимально допустимая дистанция между судами, морские мили.",
    "TCPA_LIMIT_SEC": "Предельное время до кратчайшего сближения, в пределах которого ситуация считается опасной.",
    "MANEUVER_ACTIVATION_DISTANCE_NM": "Дистанция до цели, при которой разрешается запускать маневр.",
    "COURSE_DELTA_MIN_DEG": "Минимальное проверяемое изменение курса собственного судна, градусы.",
    "COURSE_DELTA_MAX_DEG": "Максимальное проверяемое изменение курса собственного судна, градусы.",
    "COURSE_DELTA_STEP_DEG": "Шаг перебора изменения курса, градусы.",
    "SPEED_DELTA_MIN_KN": "Минимальное проверяемое изменение скорости собственного судна, узлы.",
    "SPEED_DELTA_MAX_KN": "Максимальное проверяемое изменение скорости собственного судна, узлы.",
    "SPEED_DELTA_STEP_KN": "Шаг перебора изменения скорости, узлы.",
    "MIN_ALLOWED_SPEED_KN": "Нижнее ограничение скорости собственного судна при подборе маневра.",
    "COURSE_COST_WEIGHT": "Вес изменения курса в функции стоимости маневра.",
    "SPEED_COST_WEIGHT": "Вес изменения скорости в функции стоимости маневра.",
    "MAX_TURN_RATE_DEG_PER_SEC": "Максимальная скорость изменения курса собственного судна, градусы в секунду.",
    "COURSE_RESPONSE_GAIN": "Коэффициент реакции на ошибку курса.",
    "MAX_ACCEL_KN_PER_SEC": "Максимальное изменение скорости собственного судна, узлы в секунду.",
    "RETURN_CLEARANCE_FACTOR": "Коэффициент запаса для начала возврата после прохождения CPA.",
    "ROUTE_LOOKAHEAD_NM": "Упреждающая точка на исходной линии пути для возврата.",
    "ROUTE_REJOIN_TOLERANCE_NM": "Допуск выхода на исходную линию пути, морские мили.",
    "CURRENT_X_KN": "Составляющая течения по оси X, узлы.",
    "CURRENT_Y_KN": "Составляющая течения по оси Y, узлы.",
    "OUTPUT_IMAGE_PATH": "Имя файла для сохранения графика.",
    "ENABLE_STATIC_PLOT": "Флаг показа статичного графика.",
    "ENABLE_ANIMATION": "Флаг показа анимации движения судов.",
    "ANIMATION_FRAME_STEP": "Шаг прореживания кадров анимации по расчетным точкам.",
    "ANIMATION_INTERVAL_MS": "Интервал между кадрами анимации, миллисекунды.",
    "ANIMATION_TAIL_LENGTH_SEC": "Длина отображаемого следа судов в секундах. None - показывать весь пройденный путь.",
    "OUTPUT_ANIMATION_PATH": "Путь для сохранения анимации. None - не сохранять. Поддерживаются .gif и .mp4."
}