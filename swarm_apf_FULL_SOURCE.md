# Полный исходный текст проекта swarm_apf — E0

Ниже приведено полное содержимое **каждого** созданного файла проекта, без сокращений. Папки `example_results/` и служебные файлы Python сюда не включены.

## `.gitignore`

`````text
.venv/
__pycache__/
.pytest_cache/
*.egg-info/
build/
dist/
results/
`````

## `ARCHITECTURE.md`

`````markdown
# ARCHITECTURE.md — этап E0

## Назначение

Однороботный регрессионный baseline из исходного `apf_vortex.py`.
Не реализованы рой, формация, сенсоры, многороботный цикл или физическая
динамика. Модель — точечный голономный агент на плоскости, фиксированный
нормализованный шаг по суммарному полю и неподвижные круги.

## Структура и разрешённые зависимости

```text
scenarios -> geometry
fields -> geometry
controller -> fields
planner -> controller + geometry + metrics
metrics -> geometry
visualization -> geometry (только входные объекты)
cli -> scenarios + planner + visualization
```

`fields` не знает о траектории; `controller` не знает о файлах/графиках;
`planner` не исправляет опасную команду; `metrics` не влияет на управление.
Математические функции не импортируют Matplotlib.

## Контракты

- `Obstacle(center, radius)`: 2 конечные координаты, `radius > 0`.
- `Obstacle.distance(p, agent_radius=0)`: знаковый зазор до поверхности.
- `segment_circle_clearance(a,b,o)`: минимум на **всём** отрезке.
- `attractive_force(p,g,k_att) = k_att * (g-p)`.
- `vortex_repulsive_force`: исходная радиальная величина и +90° вихрь
  для `d < d0`, с регуляризацией и ограничением возле `d=0`.
- `VortexController.direction`: единичный вектор или ноль при компенсации.
- `plan_path`: возвращает `PathResult`; `path` содержит начальную точку и лишь принятые последующие точки.
- `PathResult.iterations`: попытки; `len(path)-1`: принятые шаги.
- `PathResult.collision_segment`: отклонённый отрезок или `None`.

## Такт планировщика

1. Проверить параметры и допустимость старта/цели.
2. Если уже у цели — `SUCCESS` с нулём шагов.
3. Рассчитать силу и направление, без запасного движения к цели.
4. Предложить нормализованный шаг длины `min(step, расстояние_до_цели)`.
5. Проверить весь отрезок со всеми кругами (касание тоже коллизия).
6. При контакте завершить `COLLISION_OBSTACLE`, сохранить предложение;
   иначе принять шаг и проверить цель/стагнацию.
7. По исчерпании бюджета завершить `TIMEOUT`.

## Сохранение и отличия от исходника

Для `d >= softening`, `d < d0` и до насыщения амплитуды сохранены
формула отталкивания, знак циркуляции и сумма с линейным притяжением.
При обычном шаге далеко от цели сохраняется его нормализация.
Рядом с поверхностью ограничен `1/d`, чтобы не было разрыва и мёртвого слоя.
Случайное выталкивание в центре и случайный escape-прыжок удалены.
`is_stagnant` ищет цикличность или остановку в полном окне, не медленный
прямолинейный прогресс. Это диагностическое правило, не escape-контроллер.

## Безопасность и воспроизводимость

Нет коррекции/проекции траектории или заявления о гарантированной безопасности.
Все перемещения до принятия проверяются непрерывно. При коллизии сохраняется
непринятый отрезок; `min_clearance` включает его. Глобальная случайность не
используется и не меняется при импорте. `seed` CLI — только метаданные E0.
`config.json` сохраняет параметры и полную геометрию сцен, CSV — принятые
траектории, `results.json` — статусы/зазоры/отклонённые шаги.

`agent_radius` сейчас влияет только на геометрический зазор; формулы полей
остаются унаследованными для точечного агента. Это **не** полноценная
реализация взаимодействия робота-диска с препятствием.

## Проверенная регрессия

Установка `pip install --no-build-isolation --no-deps -e .` и
`python -m pytest -q` проверены в Python 3.13.5: **37 passed**.
Шесть штатных сценариев 1–6 сохранили исходные числа итераций:
`175, 194, 195, 124, 214, 216`; новые метрики зазора измеряют весь отрезок.
Сценарий 7 после исправления геометрии завершается `STAGNATION` за 56 шагов;
это не доказательство невозможности найти выход, а отказ данной эвристики.

## Следующий этап

E1 отдельно добавит `WorldState`, роботов-дисков, локальные наблюдения,
синхронную кинематику и непрерывную проверку пары движущихся роботов.
E0 не притворяется децентрализованной системой: один контроллер получает
список всех дисков из сцены.
`````

## `HANDOFF.md`

`````markdown
# HANDOFF.md — состояние после E0

## Выполнено

- `apf_vortex.py` разделён на geometry/fields/controller/planner/scenarios/
  metrics/visualization/cli; исходный файл оставлен источником сравнения.
- Исправлены дефекты аудита A01–A12 и A14–A15 в пределах одиночной модели:
  проверка всего отрезка, допустимость старта и цели, предел силы, отсутствие
  мёртвого слоя, корректный старт у цели, ограничение конечного шага,
  обработка нулевой силы, окно стагнации, отсутствие глобального RNG,
  валидация параметров, статус и метрики.
- A04: геометрия исходной сцены №7 изменена — старт `(3.5,3)`, проём
  между поверхностями двух ближайших кругов `0.8`; ширина для точки,
  а не для будущего робота-диска.
- A13: сцена №5 сохранена как есть и переименована по смыслу;
  ссылки на несуществующие отжиг/ослабление полей удалены.
- 37 успешных автотестов проверяют геометрию, поля, цель, отрезки,
  нулевые расстояния, стагнацию, сохранение конфигурации и воспроизводимость.

## Наблюдаемое изменение результатов

1. Ранее опасные траектории могли считаться `SUCCESS`; теперь
   обнаруженный контакт заканчивается `COLLISION_OBSTACLE`.
2. Сценарий №7 не сопоставим по числам с предыдущей версией — геометрия
   действительно изменена. Успех не гарантирован даже после исправления.
3. Вблизи границы диска силы отличаются из-за регуляризации и ограничений.
4. При точной компенсации сил возвращается `ZERO_FORCE`, а не движение
   к цели; при циклической стагнации `STAGNATION` вместо случайного прыжка.
5. Близкая цель даёт успех до шага; при точной цели `iterations=0`.
6. PNG теперь включает полные круги; число шагов и длина могут измениться.

## Фактически выполненные проверки (Python 3.13.5)

Команда `python -m pytest -q` после editable-установки: `37 passed`.
Прогон `python -m swarm_apf.cli --scenario all --out example_results`:

| Сцена | Статус | Попытки шага | Длина пути |
|---|---|---:|---:|
| 1 | SUCCESS | 175 | 14.00 |
| 2 | SUCCESS | 194 | 15.52 |
| 3 | SUCCESS | 195 | 15.60 |
| 4 | SUCCESS | 124 | 9.92 |
| 5 | SUCCESS | 214 | 17.12 |
| 6 | SUCCESS | 216 | 17.28 |
| 7 (изменена) | STAGNATION | 56 | 4.48 |

Сцены 1–6 соответствуют цифрам аудита; небольшие отличия минимального
зазора закономерны: теперь измеряется целый отрезок, а не только его вершина.
Все семь PNG были созданы; `config.json` содержит полную геометрию сцен.

## Границы подтверждения

В E0 агент — точка (`agent_radius=0` по умолчанию). Нет сравнения методов
B1/B2/B3/A1, роя, формации, локальных сенсоров или новых S01–S06 ТЗ.
Ни безопасность, ни полнота APF математически не гарантируются.

## Что делать в следующем чате (E1)

1. Зафиксировать E0 как базовую регрессию; сохранить результаты `pytest`
   и запуска `python -m swarm_apf.cli --scenario all`.
2. Развести `geometry` и будущий `physics`: для диска робота добавить
   `robot_radius`, а для пар движущихся роботов — относительную траекторию.
3. Ввести `WorldState`, `Observation`, `ControllerConfig`, локальные сенсоры
   по `R_s/R_n`, независимые команды и синхронное принятие всех шагов.
4. Проверить контракт локальности и порядок обхода ID (G08), прежде чем
   переносить `VortexController` в B1/B2.
5. Ввести *новые* S01–S06 из `PROJECT_SPEC.md`; не смешивать их со старыми
   номерами сцен из однороботной курсовой.

## Контрольные команды

```bash
python -m pip install -e ".[dev]"
python -m pytest -q
python -m swarm_apf.cli --scenario all --out results
```
`````

## `README.md`

`````markdown
# Single-agent APF + Vortex — E0

Это **не симулятор роя**. Это воспроизводимый рефакторинг исходной
`apf_vortex.py`: точечный агент, статические диски, фиксированная
циркуляция против часовой стрелки, нормализованный шаг.

## Запуск

```bash
python -m venv .venv
# Linux/macOS: source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m pytest -q
python -m swarm_apf.cli --scenario all --out results
python -m swarm_apf.cli --scenario 2 --out results/single --no-plot
```

В `results/`: `config.json` (в том числе геометрия сцен),
`results.json`, CSV траекторий и PNG.
`--seed` только записывается в конфигурацию: в E0 случайных действий нет.

## Изменения по сравнению с оригиналом

- Поле и шаг сохраняют исходную формулу в штатной области влияния,
  но вблизи поверхности сила ограничивается и не содержит мёртвого слоя.
- При почти нулевой силе нет движения «наугад» к цели.
- Перед первым шагом проверяются начало/цель, каждый отрезок — на коллизию.
- Короткий конечный шаг предотвращает перескок цели в свободном пространстве.
- Стагнация — малое итоговое смещение относительно пройденного пути за окно,
  а не малая длина отдельного шага. Случайный escape-прыжок удалён.
- Исходная сцена №7 исправлена (старт и ширина двери); в №5 название
  прямо указывает, что старт снаружи U, а не в ловушке.
- Статус `SUCCESS`, `INVALID_SCENE`, `COLLISION_OBSTACLE`, `ZERO_FORCE`,
  `STAGNATION`, `TIMEOUT`, `NUMERIC_ERROR` отделяет исходы друг от друга.

`iterations` — число *попыток* шага, включая отклонённую из-за коллизии;
число принятых шагов = `len(path)-1`. `collision_segment` сохраняет опасное
предложение отдельно. `min_clearance` учитывает в том числе отклонённый
отрезок; `null` при отсутствии препятствий в `results.json`.

Гарантий нахождения пути и предотвращения коллизий у APF нет.
Для E1 и далее см. `ARCHITECTURE.md` и `HANDOFF.md`.
`````

## `pyproject.toml`

`````toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "swarm-apf"
version = "0.1.0"
description = "E0: reproducible single-agent APF + vortex baseline"
requires-python = ">=3.11"
dependencies = ["numpy>=1.24", "matplotlib>=3.7"]

[project.optional-dependencies]
dev = ["pytest>=7.4"]

[project.scripts]
swarm-apf = "swarm_apf.cli:main"

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
testpaths = ["tests"]
`````

## `src/swarm_apf/__init__.py`

`````python
"""Single-agent APF + vortex baseline (stage E0)."""

from .geometry import Obstacle
from .planner import PathResult, plan_path

__all__ = ["Obstacle", "PathResult", "plan_path"]
`````

## `src/swarm_apf/cli.py`

`````python
"""CLI for regression experiments. No global RNG mutations."""

import argparse
import csv
import json
from pathlib import Path
from .planner import plan_path
from .scenarios import scenarios
from .visualization import save_path_figure


LEGACY_PARAMS = dict(step=0.08, max_iterations=5000, k_att=1.5,
                     k_rep=2.5, d0=2.5, k_circ=0.8, goal_tolerance=0.15)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Single-agent APF+vortex (E0)")
    parser.add_argument("--scenario", choices=["all", *map(str, range(1, 8))],
                        default="all")
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--seed", type=int, default=42,
                        help="Recorded for future experiments; E0 is deterministic")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    chosen = scenarios()
    if args.scenario != "all":
        chosen = {int(args.scenario): chosen[int(args.scenario)]}
    rows = []
    configuration = {
        "version": "0.1.0",
        "seed": args.seed,
        "uses_randomness": False,
        "params": LEGACY_PARAMS,
        "agent_radius": 0.0,
        "force_softening": 0.05,
        "max_repulsion": 10000.0,
        "epsilon_geom": 1e-9,
        "stagnation_window": 40,
        "stagnation_displacement": 0.18,
        "stagnation_ratio": 0.2,
        "scenarios": {
            str(n): {
                "name": scene.name,
                "start": scene.start,
                "goal": scene.goal,
                "obstacles": [{"center": o.center, "radius": o.radius}
                              for o in scene.obstacles],
            } for n, scene in chosen.items()
        },
    }
    with (args.out / "config.json").open("w", encoding="utf-8") as f:
        json.dump(configuration, f, indent=2, allow_nan=False)
    for number, scene in chosen.items():
        result = plan_path(scene.start, scene.goal, scene.obstacles, **LEGACY_PARAMS)
        row = dict(scenario=number, name=scene.name, status=result.status,
                   success=result.success, iterations=result.iterations,
                   accepted_steps=len(result.path) - 1,
                   path_length=result.path_length,
                   min_clearance=(result.min_clearance
                                  if result.min_clearance != float("inf") else None),
                   collision_segment=(result.collision_segment.tolist()
                                      if result.collision_segment is not None else None))
        rows.append(row)
        with (args.out / f"{number:02d}_path.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(result.path)
        if not args.no_plot:
            save_path_figure(result.path, scene.obstacles, scene.start, scene.goal,
                             args.out / f"{number:02d}_{scene.name}.png",
                             f"{scene.name}: {result.status}")
        print(f"{number}: {scene.name}: {result.status}, {result.iterations} attempts")
    with (args.out / "results.json").open("w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2, allow_nan=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
`````

## `src/swarm_apf/controller.py`

`````python
"""Stateless single-agent controller and local trajectory stagnation check."""

from dataclasses import dataclass
import numpy as np
from .fields import total_force
from .geometry import point2


@dataclass(frozen=True)
class VortexController:
    k_att: float = 1.0
    k_rep: float = 2.0
    d0: float = 2.0
    k_circ: float = 0.5

    def direction(self, position, goal, obstacles) -> np.ndarray:
        force = total_force(position, goal, obstacles,
                            self.k_att, self.k_rep, self.d0, self.k_circ)
        magnitude = float(np.linalg.norm(force))
        if not np.isfinite(magnitude):
            raise FloatingPointError("Nonfinite force")
        # No unsafe fallback towards the goal when forces cancel.
        return force / magnitude if magnitude > 1e-10 else np.zeros(2)


def is_stagnant(recent_positions, displacement_limit: float = 0.18,
                 ratio_limit: float = 0.2) -> bool:
    """Detect a stop or closed/oscillating trajectory in a FULL window.

    The caller controls window length. Slow straight motion has ratio 1,
    regardless of step size, and is never classified as stagnation.
    """
    pts = np.asarray(recent_positions, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        raise ValueError("Need at least two 2D points")
    if not np.all(np.isfinite(pts)):
        raise ValueError("Nonfinite trajectory")
    if not (np.isfinite(displacement_limit) and displacement_limit > 0
            and np.isfinite(ratio_limit) and 0 < ratio_limit < 1):
        raise ValueError("Invalid stagnation thresholds")
    distance = float(np.linalg.norm(pts[-1] - pts[0]))
    travelled = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
    return distance < displacement_limit and (
        travelled <= 1e-12 or distance / travelled < ratio_limit
    )
`````

## `src/swarm_apf/fields.py`

`````python
"""Original APF/vortex composition with bounded near-surface behavior."""

import math
import numpy as np
from .geometry import Obstacle, point2


def _parameters(k_att=None, k_rep=None, d0=None, k_circ=None,
                softening=None, max_repulsion=None) -> None:
    for name, value in (("k_att", k_att), ("k_rep", k_rep),
                        ("d0", d0), ("k_circ", k_circ),
                        ("softening", softening),
                        ("max_repulsion", max_repulsion)):
        if value is not None and not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if k_att is not None and k_att < 0:
        raise ValueError("k_att must be >= 0")
    if k_rep is not None and k_rep < 0:
        raise ValueError("k_rep must be >= 0")
    for name, value in (("d0", d0), ("softening", softening),
                        ("max_repulsion", max_repulsion)):
        if value is not None and value <= 0:
            raise ValueError(f"{name} must be > 0")


def attractive_force(position, goal, k_att: float = 1.0) -> np.ndarray:
    _parameters(k_att=k_att)
    return k_att * (point2(goal) - point2(position))


def vortex_repulsive_force(position, obstacles: tuple[Obstacle, ...],
                           k_rep: float = 2.0, d0: float = 2.0,
                           k_circ: float = 0.5,
                           softening: float = 0.05,
                           max_repulsion: float = 10000.0) -> np.ndarray:
    """CCW circulation (+90°); old formula unchanged in its regular domain.

    For d < softening clamp the inverse-distance argument; also cap the
    radial amplitude. At the exact center select a deterministic normal.
    The same expression is used on either side of the circle boundary.
    """
    _parameters(k_rep=k_rep, d0=d0, k_circ=k_circ,
                softening=softening, max_repulsion=max_repulsion)
    p = point2(position)
    result = np.zeros(2, dtype=float)
    for obs in obstacles:
        vec = p - obs.center
        rho = float(np.linalg.norm(vec))
        d = rho - obs.radius
        if d >= d0:
            continue
        radial = vec / rho if rho > 0 else np.array([1.0, 0.0])
        safe_d = max(d, min(softening, 0.5 * d0))
        amplitude = k_rep * (1.0 / safe_d - 1.0 / d0) / (safe_d**2 + 1e-10)
        amplitude = min(amplitude, max_repulsion)
        tangent = np.array([-radial[1], radial[0]])
        result += amplitude * (radial + k_circ * tangent)
    return result


def total_force(position, goal, obstacles,
                k_att: float = 1.0, k_rep: float = 2.0,
                d0: float = 2.0, k_circ: float = 0.5) -> np.ndarray:
    return (attractive_force(position, goal, k_att)
            + vortex_repulsive_force(position, obstacles, k_rep, d0, k_circ))
`````

## `src/swarm_apf/geometry.py`

`````python
"""Valid 2D circles and continuous segment–circle clearance."""

from dataclasses import dataclass
import math
import numpy as np

EPS_GEOM = 1e-9


def point2(value) -> np.ndarray:
    """Return a validated independent float vector of shape (2,)."""
    p = np.asarray(value, dtype=float)
    if p.shape != (2,) or not np.all(np.isfinite(p)):
        raise ValueError("Expected two finite coordinates")
    return p.copy()


@dataclass(frozen=True)
class Obstacle:
    center: tuple[float, float]
    radius: float

    def __post_init__(self) -> None:
        c = point2(self.center)
        r = float(self.radius)
        if not math.isfinite(r) or r <= 0:
            raise ValueError("Obstacle radius must be positive and finite")
        object.__setattr__(self, "center", (float(c[0]), float(c[1])))
        object.__setattr__(self, "radius", r)

    def distance(self, point, agent_radius: float = 0.0) -> float:
        """Signed surface clearance; 0 means touching."""
        _check_agent_radius(agent_radius)
        return float(np.linalg.norm(point2(point) - self.center) - self.radius - agent_radius)

    def contains(self, point, agent_radius: float = 0.0) -> bool:
        return self.distance(point, agent_radius) <= 0.0


def _check_agent_radius(value: float) -> None:
    if not math.isfinite(value) or value < 0:
        raise ValueError("agent_radius must be nonnegative and finite")


def segment_circle_clearance(a, b, obstacle: Obstacle,
                             agent_radius: float = 0.0) -> float:
    """Signed minimum clearance of the ENTIRE closed segment a→b."""
    _check_agent_radius(agent_radius)
    start, end = point2(a), point2(b)
    delta = end - start
    length_squared = float(np.dot(delta, delta))
    if length_squared == 0.0:
        closest = start
    else:
        tau = float(np.clip(np.dot(np.asarray(obstacle.center) - start, delta)
                            / length_squared, 0.0, 1.0))
        closest = start + tau * delta
    return float(np.linalg.norm(closest - obstacle.center)
                 - obstacle.radius - agent_radius)


def segment_hits_obstacle(a, b, obstacle: Obstacle,
                          agent_radius: float = 0.0,
                          epsilon: float = EPS_GEOM) -> bool:
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError("epsilon must be nonnegative and finite")
    return segment_circle_clearance(a, b, obstacle, agent_radius) <= epsilon
`````

## `src/swarm_apf/metrics.py`

`````python
"""Read-only numerical metrics. Never supplies forces or corrections."""

import numpy as np
from .geometry import segment_circle_clearance


def calculate_path_length(path) -> float:
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError("Expected a finite path of shape (N, 2)")
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def minimum_path_clearance(path, obstacles, agent_radius: float = 0.0) -> float:
    """Minimum continuous clearance; +inf if there are no obstacles."""
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        raise ValueError("Expected a nonempty path of shape (N, 2)")
    if not np.all(np.isfinite(points)):
        raise ValueError("Nonfinite path")
    if not obstacles:
        return float("inf")
    if len(points) == 1:
        return min(obs.distance(points[0], agent_radius) for obs in obstacles)
    return min(segment_circle_clearance(a, b, obs, agent_radius)
               for a, b in zip(points[:-1], points[1:]) for obs in obstacles)
`````

## `src/swarm_apf/planner.py`

`````python
"""Single-agent Euler planner: decide → propose → check segment → commit."""

from dataclasses import dataclass
import math
import numpy as np
from .controller import VortexController, is_stagnant
from .geometry import EPS_GEOM, Obstacle, point2, segment_circle_clearance
from .metrics import calculate_path_length, minimum_path_clearance


@dataclass
class PathResult:
    status: str
    path: np.ndarray
    iterations: int  # attempted commands, including a rejected collision step
    path_length: float
    min_clearance: float
    collision_segment: np.ndarray | None = None

    @property
    def success(self) -> bool:
        return self.status == "SUCCESS"


def plan_path(start, goal, obstacles: tuple[Obstacle, ...] | list[Obstacle], *,
              step: float = 0.05, max_iterations: int = 5000,
              k_att: float = 1.0, k_rep: float = 2.0, d0: float = 2.0,
              k_circ: float = 0.5, goal_tolerance: float = 0.1,
              agent_radius: float = 0.0, stagnation_window: int = 40,
              stagnation_displacement: float = 0.18,
              stagnation_ratio: float = 0.2) -> PathResult:
    """Preserve original normalized direction; stop on collision, no repair.

    The legacy agent is a POINT (agent_radius=0). Nonzero radius is an
    optional clearance inflation, not a new multi-robot physics engine.
    """
    p, g = point2(start), point2(goal)
    obs = tuple(obstacles)
    if not (math.isfinite(step) and step > 0
            and math.isfinite(goal_tolerance) and goal_tolerance > 0
            and math.isfinite(agent_radius) and agent_radius >= 0):
        raise ValueError("Invalid step, tolerance or agent radius")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 0:
        raise ValueError("max_iterations must be a nonnegative integer")
    if isinstance(stagnation_window, bool) or not isinstance(stagnation_window, int) or stagnation_window < 2:
        raise ValueError("stagnation_window must be an integer >= 2")
    if not (math.isfinite(stagnation_displacement) and stagnation_displacement > 0
            and math.isfinite(stagnation_ratio) and 0 < stagnation_ratio < 1):
        raise ValueError("Invalid stagnation parameters")
    controller = VortexController(k_att, k_rep, d0, k_circ)
    # Validate coefficients even if the initial state needs zero steps.
    from .fields import _parameters
    _parameters(k_att=k_att, k_rep=k_rep, d0=d0, k_circ=k_circ)

    path = [p.copy()]
    clearance = minimum_path_clearance(np.asarray(path), obs, agent_radius)

    def finish(status, count, collision=None) -> PathResult:
        return PathResult(status, np.asarray(path), count,
                          calculate_path_length(np.asarray(path)),
                          clearance, collision)

    # Tangency is invalid even if a path starts exactly at the goal.
    if any(o.distance(p, agent_radius) <= EPS_GEOM or
           o.distance(g, agent_radius) <= EPS_GEOM for o in obs):
        return finish("INVALID_SCENE", 0)
    if np.linalg.norm(p - g) <= goal_tolerance:
        return finish("SUCCESS", 0)

    for i in range(1, max_iterations + 1):
        try:
            direction = controller.direction(p, g, obs)
            if not np.all(np.isfinite(direction)):
                return finish("NUMERIC_ERROR", i)
            if np.linalg.norm(direction) <= 1e-10:
                return finish("ZERO_FORCE", i)
            # Avoid the old free-space overshoot (D04); otherwise same step.
            actual_step = min(step, float(np.linalg.norm(g - p)))
            candidate = p + actual_step * direction
            if not np.all(np.isfinite(candidate)):
                return finish("NUMERIC_ERROR", i)
            segment_gap = min((segment_circle_clearance(p, candidate, o, agent_radius)
                               for o in obs), default=float("inf"))
        except (FloatingPointError, OverflowError):
            return finish("NUMERIC_ERROR", i)
        clearance = min(clearance, segment_gap)
        if segment_gap <= EPS_GEOM:
            return finish("COLLISION_OBSTACLE", i,
                          np.vstack([p, candidate]))
        p = candidate
        path.append(p.copy())
        if np.linalg.norm(p - g) <= goal_tolerance:
            return finish("SUCCESS", i)
        if len(path) > stagnation_window:
            if is_stagnant(path[-(stagnation_window + 1):],
                           stagnation_displacement, stagnation_ratio):
                return finish("STAGNATION", i)
    return finish("TIMEOUT", max_iterations)
`````

## `src/swarm_apf/scenarios.py`

`````python
"""Seven legacy single-point scenarios, with repaired #7 geometry."""

from dataclasses import dataclass
import numpy as np
from .geometry import Obstacle


@dataclass(frozen=True)
class Scenario:
    name: str
    start: tuple[float, float]
    goal: tuple[float, float]
    obstacles: tuple[Obstacle, ...]


def circle_line_x(xs, y, radius) -> tuple[Obstacle, ...]:
    return tuple(Obstacle((float(x), y), radius) for x in xs)


def circle_line_y(x, ys, radius) -> tuple[Obstacle, ...]:
    return tuple(Obstacle((x, float(y)), radius) for y in ys)


def scenarios() -> dict[int, Scenario]:
    """Preserve old 1–6 as regression scenes, not swarm specification S01–S06."""
    one = (Obstacle((5.0, 5.0), 1.5),)
    two = (Obstacle((4.0, 5.0), 1.2), Obstacle((6.0, 5.0), 1.2))
    corridor = circle_line_x(np.linspace(1, 9, 9), 7.0, 0.8) + \
               circle_line_x(np.linspace(1, 9, 9), 3.0, 0.8)
    u = (circle_line_x(np.linspace(2, 8, 10), 3.0, 0.7)
         + circle_line_y(2.0, np.linspace(3, 9, 12), 0.7)
         + circle_line_y(8.0, np.linspace(3, 9, 12), 0.7))
    cluster = tuple(Obstacle(c, r) for c, r in [
        ((4, 3), 1.2), ((6, 3.5), 1.0), ((3.5, 5.5), 1.0),
        ((5.5, 5), 1.3), ((4.5, 7), 1.1), ((6.5, 6.5), 1.0),
        ((7, 4), 0.9), ((5, 4), 0.8), ((2, 1), 1.0), ((8, 8), 1.0),
    ])
    # Legacy #7 started INSIDE a circle, and its doorway was sealed.
    # Starting at (3.5,3) avoids the internal disk at (2.5,3), R=.8.
    # Door disks at y=2.1 and 3.9 leave a surface gap 0.8 for a point.
    room = (circle_line_y(1, np.linspace(0, 6, 13), 0.5)
            + circle_line_x(np.linspace(1, 5, 9), 6, 0.5)
            + circle_line_x(np.linspace(1, 5, 9), 0, 0.5)
            + circle_line_y(5, np.linspace(3.9, 6, 5), 0.5)
            + circle_line_y(5, np.linspace(0, 2.1, 5), 0.5)
            + (Obstacle((2.5, 3), 0.8), Obstacle((7, 2), 0.7),
               Obstacle((8, 4), 0.7)))
    return {
        1: Scenario("free_space", (0, 0), (10, 10), ()),
        2: Scenario("single_obstacle", (0, 0), (10, 10), one),
        3: Scenario("two_obstacles", (0, 0), (10, 10), two),
        4: Scenario("narrow_corridor", (0, 5), (10, 5), corridor),
        5: Scenario("u_outline_outside_start", (5, 2), (5, 9), u),
        6: Scenario("obstacle_cluster", (0, 0), (10, 10), cluster),
        7: Scenario("room_with_open_door", (3.5, 3), (10, 3), room),
    }
`````

## `src/swarm_apf/visualization.py`

`````python
"""Headless PNG rendering; not imported by the planner/controller."""

from pathlib import Path
import numpy as np


def save_path_figure(path, obstacles, start, goal, destination, title="APF + Vortex"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = np.asarray(path, dtype=float)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.3)
    for obs in obstacles:
        ax.add_patch(plt.Circle(obs.center, obs.radius, color="gray", alpha=0.5))
    ax.plot(path[:, 0], path[:, 1], "b-", lw=2, label="Path")
    ax.scatter([start[0]], [start[1]], c="green", label="Start", zorder=4)
    ax.scatter([goal[0]], [goal[1]], c="red", marker="*", s=160,
               label="Goal", zorder=4)
    # Include full obstacle disks, not just their centers or visited positions.
    xs = [float(start[0]), float(goal[0]), *path[:, 0].tolist()]
    ys = [float(start[1]), float(goal[1]), *path[:, 1].tolist()]
    for obs in obstacles:
        xs += [obs.center[0] - obs.radius, obs.center[0] + obs.radius]
        ys += [obs.center[1] - obs.radius, obs.center[1] + obs.radius]
    ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
    ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)
    ax.set(xlabel="X", ylabel="Y", title=title)
    ax.legend()
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=150, bbox_inches="tight")
    plt.close(fig)
`````

## `tests/test_cli.py`

`````python
import json
from swarm_apf.cli import main


def test_cli_results_are_reproducible_and_record_geometry(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    assert main(["--scenario", "1", "--out", str(a), "--no-plot"]) == 0
    assert main(["--scenario", "1", "--out", str(b), "--no-plot"]) == 0
    assert (a / "results.json").read_bytes() == (b / "results.json").read_bytes()
    assert (a / "01_path.csv").read_bytes() == (b / "01_path.csv").read_bytes()
    config = json.loads((a / "config.json").read_text())
    assert config["scenarios"]["1"]["start"] == [0, 0]
    assert config["uses_randomness"] is False
`````

## `tests/test_fields.py`

`````python
import numpy as np
import pytest
from swarm_apf.geometry import Obstacle
from swarm_apf.fields import attractive_force, vortex_repulsive_force, total_force


def test_attraction_and_no_obstacle_force_far_away():
    assert np.allclose(attractive_force((0, 0), (2, 0), 1.5), (3, 0))
    obs = Obstacle((0, 0), 1)
    assert np.array_equal(vortex_repulsive_force((4, 0), [obs]), [0, 0])
    assert np.allclose(total_force((0, 0), (2, 0), [], k_att=1.5), [3, 0])


def test_vortex_rotates_counterclockwise_and_radial_only_when_zero():
    obs = Obstacle((0, 0), 1)
    force = vortex_repulsive_force((2, 0), [obs], k_circ=.8)
    assert force[0] > 0
    assert force[1] == pytest.approx(.8 * force[0])
    radial = vortex_repulsive_force((2, 0), [obs], k_circ=0)
    assert radial[0] > 0
    assert radial[1] == pytest.approx(0)


def test_near_surface_and_center_are_finite_and_not_dead():
    obs = Obstacle((0, 0), 1)
    for d in [2e-6, 0.5e-6, 0, -1e-7, -1]:
        f = vortex_repulsive_force((1 + d, 0), [obs])
        assert np.all(np.isfinite(f))
        assert f[0] > 0
    center = vortex_repulsive_force((0, 0), [obs])
    assert np.all(np.isfinite(center))
    assert center[0] > 0


def test_symmetric_radial_forces_cancel():
    obstacles = [Obstacle((-2, 0), .5), Obstacle((2, 0), .5)]
    force = vortex_repulsive_force((0, 0), obstacles, k_circ=0)
    assert np.allclose(force, [0, 0])


def test_invalid_force_parameters():
    with pytest.raises(ValueError):
        vortex_repulsive_force((0, 0), [], d0=0)
    with pytest.raises(ValueError):
        attractive_force((0, 0), (1, 0), k_att=-1)
`````

## `tests/test_geometry.py`

`````python
import numpy as np
import pytest
from swarm_apf.geometry import (Obstacle, segment_circle_clearance,
                                segment_hits_obstacle)


def test_signed_distance_and_boundary():
    obs = Obstacle((0, 0), 1)
    assert [obs.distance(p) for p in [(2, 0), (1, 0), (0, 0)]] == [1, 0, -1]
    assert not obs.contains((2, 0))
    assert obs.contains((1, 0))
    assert obs.contains((0, 0))


@pytest.mark.parametrize("center,radius", [((0,), 1), ((0, 0), 0),
                                             ((0, 0), -1), ((float("nan"), 0), 1)])
def test_invalid_obstacles(center, radius):
    with pytest.raises(ValueError):
        Obstacle(center, radius)


def test_segment_crosses_disk_even_if_endpoints_are_safe():
    obs = Obstacle((0, 0), 1)
    assert obs.distance((-2.5, 0)) > 0
    assert obs.distance((2.5, 0)) > 0
    assert segment_circle_clearance((-2.5, 0), (2.5, 0), obs) == -1
    assert segment_hits_obstacle((-2.5, 0), (2.5, 0), obs)


def test_tangent_zero_length_and_agent_radius():
    obs = Obstacle((0, 0), 1)
    assert segment_circle_clearance((-2, 1), (2, 1), obs) == pytest.approx(0)
    assert segment_hits_obstacle((-2, 1), (2, 1), obs)
    assert segment_circle_clearance((2, 0), (2, 0), obs) == pytest.approx(1)
    assert segment_circle_clearance((2, 0), (2, 0), obs, .5) == pytest.approx(.5)
    assert not segment_hits_obstacle((2, 0), (2, 0), obs)
`````

## `tests/test_planner.py`

`````python
import numpy as np
import pytest
from swarm_apf.controller import is_stagnant
from swarm_apf.geometry import Obstacle
from swarm_apf.metrics import calculate_path_length, minimum_path_clearance
from swarm_apf.planner import plan_path


def test_already_at_goal_without_step():
    result = plan_path((0, 0), (0, 0), [])
    assert result.success and result.iterations == 0
    assert result.path.shape == (1, 2) and result.path_length == 0


def test_goal_inside_tolerance_already_succeeds():
    result = plan_path((0, 0), (.03, 0), [], step=.05, goal_tolerance=.1)
    assert result.success and result.iterations == 0


def test_no_free_space_overshoot_for_small_tolerance():
    result = plan_path((0, 0), (.04, 0), [], step=.08, goal_tolerance=.01)
    assert result.success and result.iterations == 1
    assert np.allclose(result.path[-1], [.04, 0])


def test_collision_on_segment_stops_before_invalid_step():
    result = plan_path((-2.5, 0), (2.5, 0), [Obstacle((0, 0), 1)],
                       step=5, d0=1, max_iterations=1)
    assert result.status == "COLLISION_OBSTACLE"
    assert result.iterations == 1 and len(result.path) == 1
    assert result.collision_segment is not None
    assert result.min_clearance < 0


def test_initial_collision_and_goal_collision_invalid():
    disk = [Obstacle((0, 0), 1)]
    assert plan_path((0, 0), (3, 0), disk).status == "INVALID_SCENE"
    assert plan_path((3, 0), (0, 0), disk).status == "INVALID_SCENE"
    assert plan_path((1, 0), (3, 0), disk).status == "INVALID_SCENE"


def test_force_cancellation_has_no_unsafe_goal_fallback():
    result = plan_path((0, 0), (2, 0), [], k_att=0, max_iterations=10)
    assert result.status == "ZERO_FORCE" and result.iterations == 1
    assert len(result.path) == 1


def test_stagnation_detects_oscillation_and_stop_not_slow_progress():
    oscillation = np.array([[0, 0], [.08, 0]] * 21)
    assert is_stagnant(oscillation)
    assert is_stagnant(np.zeros((41, 2)))
    slow = np.array([[i * .005, 0] for i in range(41)])
    assert not is_stagnant(slow)
    assert plan_path((0, 0), (2, 0), [], step=.005,
                     max_iterations=42, goal_tolerance=.001).status == "TIMEOUT"


def test_actual_iteration_count_and_timeout():
    result = plan_path((0, 0), (1, 0), [], max_iterations=0)
    assert result.status == "TIMEOUT" and result.iterations == 0
    assert len(result.path) == 1


def test_same_inputs_independent_of_global_rng_and_other_runs():
    kwargs = dict(step=.08, max_iterations=200, goal_tolerance=.15)
    obstacles = [Obstacle((5, 5), 1.5)]
    a = plan_path((0, 0), (10, 10), obstacles, **kwargs)
    np.random.seed(735)
    np.random.random(1000)
    plan_path((0, 0), (20, 0), [], max_iterations=20)
    b = plan_path((0, 0), (10, 10), obstacles, **kwargs)
    assert a.status == b.status
    assert a.iterations == b.iterations
    assert np.array_equal(a.path, b.path)
    assert a.path_length == b.path_length


def test_length_and_clearance_metrics():
    path = np.array([[0, 0], [3, 4]])
    assert calculate_path_length(path) == pytest.approx(5)
    assert minimum_path_clearance(path, [Obstacle((1.5, 2), .3)]) == pytest.approx(-.3)


@pytest.mark.parametrize("kwargs", [{"step": 0}, {"goal_tolerance": 0},
                                    {"max_iterations": -1}, {"d0": 0}])
def test_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        plan_path((0, 0), (1, 0), [], **kwargs)


def test_exact_force_cancellation_never_tunnels_toward_goal():
    # Goal lies beyond the disk, but its attraction cancels radial repulsion.
    gap = .75
    k_rep = 2 * (gap**2 + 1e-10) / (1 / gap - 1 / 2)
    result = plan_path((0, 0), (2, 0), [Obstacle((1, 0), .25)],
                       k_att=1, k_rep=k_rep, k_circ=0, d0=2,
                       max_iterations=10)
    assert result.status == "ZERO_FORCE"
    assert len(result.path) == 1
`````

## `tests/test_scenarios.py`

`````python
from swarm_apf.geometry import segment_circle_clearance
from swarm_apf.metrics import minimum_path_clearance
import pytest
from swarm_apf.scenarios import scenarios
from swarm_apf.planner import plan_path


def test_all_legacy_scenes_have_valid_endpoints():
    data = scenarios()
    assert len(data) == 7
    for scene in data.values():
        assert all(o.distance(scene.start) > 0 and o.distance(scene.goal) > 0
                   for o in scene.obstacles), scene.name


def test_repaired_room_has_positive_door_gap():
    scene = scenarios()[7]
    assert scene.start == (3.5, 3)
    right_wall = [o for o in scene.obstacles if o.center[0] == 5]
    # Segment across the opening has strictly positive clearance.
    assert min(segment_circle_clearance((4.9, 3), (5.1, 3), o)
               for o in right_wall) > 0
    # A complete straight line from start to goal is geometrically possible.
    assert minimum_path_clearance([scene.start, scene.goal], scene.obstacles) > 0


def test_free_space_regression_reaches_goal():
    scene = scenarios()[1]
    result = plan_path(scene.start, scene.goal, scene.obstacles, step=.08,
                       max_iterations=5000, k_att=1.5, k_rep=2.5, d0=2.5,
                       k_circ=.8, goal_tolerance=.15)
    assert result.success
    assert result.iterations == 175  # legacy behavior unchanged here


@pytest.mark.parametrize("number,expected_steps", [
    (1, 175), (2, 194), (3, 195),
    (4, 124), (5, 214), (6, 216),
])
def test_legacy_regression_scenarios(number, expected_steps):
    scene = scenarios()[number]
    result = plan_path(scene.start, scene.goal, scene.obstacles, step=.08,
                       max_iterations=5000, k_att=1.5, k_rep=2.5, d0=2.5,
                       k_circ=.8, goal_tolerance=.15)
    assert result.success and result.iterations == expected_steps
    assert result.path_length == pytest.approx(expected_steps * .08)
    assert result.min_clearance > 0
`````
