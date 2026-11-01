# Swarm APF — симулятор на Python

Собраны E0 (однороботный APF/vortex) и E1 (синхронный рой роботов-дисков).
В E1 доступны диагностический GoalController и экспериментальный Boids.
Контроллеры A–D и экспериментальный runner пока не интегрированы.

## Установка

Требуется Python 3.11 или новее.

```bash
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m pytest -q
```

## Рой E1

```bash
python -m swarm_apf.swarm_cli --robots 5 --out results/e1
python -m swarm_apf.swarm_cli --robots 10 --controller boids --out results/boids
python -m swarm_apf.swarm_cli --scenario obstacle --out results/collision
```

Параметры: 5–20 роботов, --max-steps (по умолчанию 600), --no-plot.
Экспорт: history.csv (только принятые состояния), summary.json (конфигурация,
контроллер, исходы и отклонённый шаг), trajectory.png.
Сценарий obstacle с GoalController специально заканчивается столкновением:
этот контроллер движется прямо к цели и не обходит препятствия.
Boids по умолчанию также не включает отталкивание от препятствий.
Статус эксперимента хранится в JSON; нормальный выход CLI не означает SUCCESS.

## Однороботная регрессия E0

```bash
python -m swarm_apf.cli --scenario all --out results/e0
```

## Архитектура

Рабочий код находится в src/swarm_apf, тесты — в tests.

- geometry, fields, controller, planner, scenarios, metrics, visualization, cli: E0.
- models: неизменяемые снимки мира и локальные наблюдения.
- sensing: единственная граница между полным миром и наблюдениями агента.
- swarm_controller, boids: команды по одному локальному наблюдению.
- physics: непрерывная проверка движущихся дисков и круговых препятствий.
- swarm: общий снимок, синхронный расчёт, атомарное принятие шага, экспорт.
- swarm_cli, swarm_visualization: запуск и офлайн-графики.

Симулятор не исправляет опасные команды: при контакте отклоняется весь шаг.
Ограничение скорости проверяется, а не обрезается. Гарантий безопасности
и сходимости у контроллеров нет.

## Источники и следующие этапы

Файлы Python в корне — исторические исходные материалы; запускать следует
пакет, а не эти файлы напрямую. Документы BASELINE_* и ARCHITECTURE (4).md
описывают также ещё не интегрированные этапы. swarm_controller, swarm_cli,
swarm_visualization и интеграционные тесты восстановлены при сборке.
В models/sensing добавлена передача известного robot_radius для Boids.

Далее: реализация A и formation B по спецификациям, подключение C/D,
интеграция experiment_runner с проверкой происхождения A/B, сценарии E2.
Нельзя считать реконструированные реализации исходными замороженными baseline
без проверки их исходников и контрольных хешей.

Проверено при сборке: 53 теста; E1 free (5 роботов) — SUCCESS за 165 шагов;
Boids free (10) — SUCCESS за 167; obstacle — COLLISION_OBSTACLE на попытке 63.
