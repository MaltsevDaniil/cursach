# Swarm APF — симулятор на Python

Собраны E0 (однороботный APF/vortex) и E1 (синхронный рой роботов-дисков).
В E1 доступны диагностический GoalController и экспериментальный Boids.
Добавлены A (локальный APF) и B (Formation APF), восстановленные по спецификациям.
C (фиксированная циркуляция) и D (адаптивные режимы) подключены к E1.
Экспериментальный runner E1 интегрирован. Сценарии E2 доступны через отдельный experiment_e2.

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

## APF A и формация B

```bash
python -m swarm_apf.swarm_cli --controller A --robots 5 --out results/a
python -m swarm_apf.swarm_cli --controller B --robots 5 --out results/b_line
python -m swarm_apf.swarm_cli --controller B --robots 10 --formation V --out results/b_v
python -m swarm_apf.swarm_cli --controller B --scenario obstacle --out results/b_obstacle
```

A использует индивидуальные конечные цели, радиальные отталкивания и
нормализацию суммарной силы. B отслеживает фиксированные слоты движущейся
опорной точки; line и V задают геометрию. Опорный план согласован до старта,
не зависит от текущих позиций роя и не обходит препятствия. Поэтому B требует
общего времени и reference, а не является автономным согласованием формации.
Условие SUCCESS проверяется по конечным индивидуальным слотам.

Версии A-RECONSTRUCTED-1.0.0 и B-FORMATION-RECONSTRUCTED-1.0.0 обозначают
новые реализации формул BASELINE_A/B, а не восстановление исходных файлов
побайтно. Старые lock-файлы отсутствуют; эквивалентность замороженным
историческим реализациям не подтверждена. summary.json содержит версии,
параметры и происхождение, для B — также план и раскладку ID.
CLI-сцены A и B имеют разные старты: это демонстрации, не сравнительный benchmark.

## Вихревой C и адаптивный D

```bash
python -m swarm_apf.swarm_cli --controller C --scenario obstacle --out results/c
python -m swarm_apf.swarm_cli --controller D --scenario obstacle --out results/d
python -m swarm_apf.swarm_cli --controller D --ablation no_vortex --out results/d_no_vortex
```

C/D перенесены из исходных файлов репозитория без изменения формул.
Каждому ID назначается отдельный контроллер. При успешном reset симулятора
память D очищается. Общий план и слоты предоставляет восстановленный B;
это происхождение явно записано в summary.json.

D переключает NORMAL/PASSAGE/RECOVERY по локальным зазорам и прогрессу.
Абляции: full, no_weight_adaptation, no_vortex, no_formation_restoration.
Ключ --ablation применяется только к D. Отталкивания не ослабляются;
адаптируются вес формации и циркуляция. Гарантий выхода из ловушек нет.

Дополнительно D сохраняет modes.csv: режим, причина переключения,
коэффициенты, зазоры, предложенная команда и accepted. Запись относится к
попытке команды; последняя попытка может быть отклонена физикой. При нуле
попыток файл содержит только заголовок. Метаданные сохраняют параметры
каждого контроллера, общий план и раскладку.

## Пакетные эксперименты E1

Быстрый проверочный прогон (папка назначения должна быть новой):

```bash
python -m swarm_apf.experiment_runner --allow-contract-proxies --scenarios free --robots 5 --seeds 0 --max-steps 400 --out results/benchmark_01
```

Все пять сцен, 5/10/20 роботов, по десять seed (600 запусков):

```bash
python -m swarm_apf.experiment_runner --allow-contract-proxies --out results/benchmark_full_01
```

Сцены E1: free, single_obstacle, symmetric_obstacles, open_corridor,
offset_cluster. Можно ограничить --algorithms C D, --scenarios, --robots,
--seeds и --max-steps. Для C/D флаг --allow-contract-proxies не требуется,
но восстановленное происхождение геометрии B всё равно сохраняется.

**Флаг --allow-contract-proxies явно разрешает восстановленные A/B.**
Без него runner отказывается запускать A/B даже при наличии модулей.
Это предварительные эксперименты, не результаты оригинальных замороженных
baseline. Флаг не заменяет их проверку по отсутствующим историческим lock-файлам.

Каждый метод получает одинаковые начальные позиции, конечные цели,
геометрию, физику и seed для данного случая; scene_hash проверяет их тождество.
Параметры методов фиксированы. Сохраняются:

- manifest.json: настройки, версии среды, хеши исходников, статус COMPLETE/IN_PROGRESS.
- scenes/: точная геометрия и хеш каждого случая.
- runs.csv, aggregate.csv и aggregate.json: исходы и агрегаты.
- runs/: история, summary.json, ошибки формации, тайминги, метрики роботов,
  коллизии, отклонённые шаги; для D — mode_log.csv.

TIMEOUT учитывается отдельно, время завершения определено только при SUCCESS.
Метрики зазора включают отклонённый шаг. Тайминги не воспроизводятся побайтно;
траектории при одинаковых исходниках и условиях воспроизводятся.
--expect-code-sha позволяет проверить сводный хеш исходников перед запуском.
Существующие результаты не перезаписываются. IN_PROGRESS означает незавершённый
набор. Физическая допустимость стартов/целей проверяется до начала пакета;
геометрическую достижимость пути E1 не доказывает.

## Сценарии E2 — отдельная реконструированная версия

```bash
python -m swarm_apf.experiment_e2 --preflight-only --out results/e2_audit
python -m swarm_apf.experiment_e2 --allow-contract-proxies --scenarios free --robots 5 --seeds 0 --out results/e2_quick
python -m swarm_apf.experiment_e2 --scenarios blocked_room --stress-only --preflight-only --out results/e2_stress_audit
```

Основные карты: free, single_obstacle, obstacle_cluster, narrow_corridor,
u_trap, room_exit. Дополнительная two_obstacles пишется в diagnostic_*;
blocked_room требует --stress-only и пишет stress_*, не смешиваясь с основными
агрегатами. По умолчанию N=5/10/20, seed=0; остальные seed задаются --seeds.

Версия SCENARIO-E2-RECONSTRUCTED-1.0.0 намеренно отличается от исторического E2:
координаты U и кластера заданы заново, поскольку в документе нет полного списка.
Левая стена комнаты перенесена с x=-17 на x=-18: при старом положении крайний
робот N=20 V пересекал стену. Дверь, коридор и остальные параметры сохранены
по опубликованной геометрии. Не объединяйте эти результаты с историческим E2.

План (-10,0)→(10,0), скорость .5, spacing=.65, dt=.05, vmax=.8,
лимит 1600 попыток. Каждая карта сначала проходит проверку стартов/целей и
консервативную 4-связную сетку .1 с расширением препятствий на радиус диска
и половину диагонали ячейки. Проверяются также соединения конечных точек с
сеткой. Положительный результат подтверждает путь отдельного диска, не
совместный безопасный план роя. Отрицательный может быть неокончательным.
Коридор открыт с концов, внешний обход разрешён. Слишком крупный диск
вызывает SCENE_UNSUITABLE, отсутствие сертификата основной карты —
SCENE_UNCERTIFIED. Это ошибки геометрии, не неудачи контроллера.

Предварительный аудит сохраняет manifest.json, preflight.json, scenes/ и
не запускает контроллеры. Benchmark использует те же данные и физику E1.
Все папки вывода должны быть новыми. Старые команды --config-json из
EXPERIMENTS (1).md не поддерживаются: используйте команды выше.
PNG-предпросмотр карт и конфигурации JSON пока не реализованы.

## Однороботная регрессия E0

```bash
python -m swarm_apf.cli --scenario all --out results/e0
```

## Архитектура

Рабочий код находится в src/swarm_apf, тесты — в tests.

- geometry, fields, controller, planner, scenarios, metrics, visualization, cli: E0.
- models: неизменяемые снимки мира и локальные наблюдения.
- sensing: единственная граница между полным миром и наблюдениями агента.
- multi_apf: локальный APF A.
- formation: геометрия слотов, опорный план, контроллер B и диспетчер по ID.
- formation_metrics: офлайн RMS, средняя, максимальная и очищенная от сдвига ошибка.
- vortex_c, adaptive_d: локальные C и D.
- experiment_runner: факторный стенд E1, происхождение, метрики и экспорт.
- controller_team: экземпляры C/D по ID, сброс памяти и экспорт режимов.
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

Далее: полный сравнительный прогон, графические отчёты и конфигурации JSON.
Для исторического сравнения по-прежнему нужны исходные A/B и их lock-файлы.

Проверено при сборке: 77 тестов; аудит E2 — 18 основных случаев (6 карт × 3 N), seed=0; пакет E1 free N=5 seed=0 — SUCCESS для A/B/C/D; C obstacle (5) — SUCCESS за 231 шаг,
D obstacle (5) — SUCCESS за 238 шагов; A free (5) — SUCCESS за 165 шагов;
B free V (10) — SUCCESS за 231 шаг; E1 free (5 роботов) — SUCCESS за 165 шагов;
Boids free (10) — SUCCESS за 167; obstacle — COLLISION_OBSTACLE на попытке 63.
