from dataclasses import dataclass

from ship_model import ShipState


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    name: str
    own_initial_state: ShipState
    target_initial_state: ShipState
    expected_maneuver: bool | None
    comment: str


def get_control_scenarios() -> list[Scenario]:
    return [
        Scenario(
            scenario_id="S01",
            name="Параллельное движение, безопасное расстояние",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.8, 0.0, 0.0, 12.0),
            expected_maneuver=False,
            comment="Относительная скорость близка к нулю, дистанция больше безопасной."
        ),
        Scenario(
            scenario_id="S02",
            name="Параллельное движение, разные скорости",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 10.0),
            target_initial_state=ShipState(1.2, 0.0, 0.0, 8.0),
            expected_maneuver=False,
            comment="Курсы параллельны, поперечная дистанция достаточна."
        ),
        Scenario(
            scenario_id="S03",
            name="Пересечение линий пути без опасного сближения во времени",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(3.0, 3.0, 270.0, 6.0),
            expected_maneuver=False,
            comment="Линии пути пересекаются, но суда приходят в точку пересечения в разное время."
        ),
        Scenario(
            scenario_id="S04",
            name="Цель уже расходится, TCPA меньше нуля",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.0, -1.0, 180.0, 12.0),
            expected_maneuver=False,
            comment="Кратчайшее сближение уже пройдено."
        ),
        Scenario(
            scenario_id="S05",
            name="Опасная точка за пределами временного окна",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.0, 20.0, 180.0, 12.0),
            expected_maneuver=False,
            comment="Сближение возможно, но TCPA выходит за пределы заданного окна."
        ),
        Scenario(
            scenario_id="S06",
            name="Близкое параллельное движение с безопасной DCPA",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 10.0),
            target_initial_state=ShipState(0.6, 0.0, 0.0, 10.0),
            expected_maneuver=False,
            comment="DCPA больше безопасной дистанции."
        ),
        Scenario(
            scenario_id="S07",
            name="Встречные курсы, лобовое сближение",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.0, 3.0, 180.0, 12.0),
            expected_maneuver=True,
            comment="Опасное встречное сближение."
        ),
        Scenario(
            scenario_id="S08",
            name="Пересечение справа, одновременный приход",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(1.333, 2.0, 270.0, 8.0),
            expected_maneuver=True,
            comment="Цель идет справа налево, ожидается опасное сближение."
        ),
        Scenario(
            scenario_id="S09",
            name="Пересечение слева, одновременный приход",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(-1.333, 2.0, 90.0, 8.0),
            expected_maneuver=True,
            comment="Цель идет слева направо, ожидается опасное сближение."
        ),
        Scenario(
            scenario_id="S10",
            name="Собственное судно догоняет цель",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 14.0),
            target_initial_state=ShipState(0.0, 1.0, 0.0, 8.0),
            expected_maneuver=True,
            comment="Опасное сближение при обгоне."
        ),
        Scenario(
            scenario_id="S11",
            name="Цель догоняет собственное судно",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 8.0),
            target_initial_state=ShipState(0.0, -1.0, 0.0, 14.0),
            expected_maneuver=True,
            comment="Цель быстрее и идет сзади."
        ),
        Scenario(
            scenario_id="S12",
            name="Диагональное сближение",
            own_initial_state=ShipState(0.0, 0.0, 45.0, 12.0),
            target_initial_state=ShipState(2.0, 0.0, 315.0, 12.0),
            expected_maneuver=True,
            comment="Суда сходятся по диагональным направлениям."
        ),
        Scenario(
            scenario_id="S13",
            name="Перпендикулярное сближение на больших скоростях",
            own_initial_state=ShipState(-1.0, 0.0, 90.0, 15.0),
            target_initial_state=ShipState(0.0, 1.0, 180.0, 15.0),
            expected_maneuver=True,
            comment="Быстрое пересечение курсов."
        ),
        Scenario(
            scenario_id="S14",
            name="Медленное встречное сближение",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 6.0),
            target_initial_state=ShipState(0.0, 1.0, 180.0, 6.0),
            expected_maneuver=True,
            comment="Опасное сближение при малых скоростях."
        ),
        Scenario(
            scenario_id="S15",
            name="Почти опасное сближение, DCPA ниже безопасной",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.4, 2.0, 180.0, 12.0),
            expected_maneuver=True,
            comment="DCPA меньше безопасной дистанции."
        ),
        Scenario(
            scenario_id="S16",
            name="Опасность появляется позже",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.0, 5.0, 180.0, 12.0),
            expected_maneuver=True,
            comment="На старте дистанция больше зоны активации, но позже возникает опасное сближение."
        ),
        Scenario(
            scenario_id="S17",
            name="Неподвижная цель на пути",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.0, 2.0, 0.0, 0.0),
            expected_maneuver=True,
            comment="Цель неподвижна, расположена на исходной линии пути."
        ),
        Scenario(
            scenario_id="S18",
            name="Цель впереди и уходит быстрее",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 8.0),
            target_initial_state=ShipState(0.0, 1.0, 0.0, 14.0),
            expected_maneuver=False,
            comment="Цель впереди и удаляется."
        ),
        Scenario(
            scenario_id="S19",
            name="Почти нулевая относительная скорость",
            own_initial_state=ShipState(0.0, 0.0, 90.0, 10.0),
            target_initial_state=ShipState(1.0, 0.0, 90.0, 10.0),
            expected_maneuver=False,
            comment="Суда идут одинаковым курсом и скоростью."
        ),
        Scenario(
            scenario_id="S20",
            name="Начальная дистанция меньше безопасной",
            own_initial_state=ShipState(0.0, 0.0, 0.0, 12.0),
            target_initial_state=ShipState(0.1, 0.0, 0.0, 12.0),
            expected_maneuver=None,
            comment="Нештатный случай. Безопасная дистанция уже нарушена в начальный момент."
        ),
    ]