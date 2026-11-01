#!/usr/bin/env python3
"""
Реализация алгоритма поиска пути методом вихревых полей (Vortex Fields / Circulation Fields)
Курсовая работа: Изучение и реализация алгоритмов поиска путей с потенциальными полями
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

# Настройка для воспроизводимости результатов
np.random.seed(42)


class Obstacle:
    """Класс для представления препятствия (круг)"""

    def __init__(self, center, radius):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)

    def distance(self, point):
        """Расстояние от точки до поверхности препятствия"""
        return np.linalg.norm(point - self.center) - self.radius

    def contains(self, point):
        """Проверка, находится ли точка внутри препятствия"""
        return self.distance(point) <= 0.0


def attractive_force(position, goal, k_att=1.0):
    """
    Притягивающая сила к цели
    F_att = -k_att * (q - q_goal)
    """
    return -k_att * (position - goal)


def vortex_repulsive_force(position, obstacles, k_rep=2.0, d0=2.0, k_circ=0.5):
    """
    Отталкивающая сила с добавлением вихревой (циркуляционной) составляющей.

    Радиальная компонента: отталкивает от препятствия по нормали.
    Тангенциальная компонента: создаёт вихрь вокруг препятствия,
    помогая избежать лобового столкновения и застревания.

    Параметры:
    - position: текущая позиция (x, y)
    - obstacles: список препятствий
    - k_rep: коэффициент радиального отталкивания
    - d0: радиус влияния препятствия
    - k_circ: коэффициент циркуляционной составляющей (направление фиксировано против часовой стрелки)
    """
    total_force = np.zeros(2)

    for obstacle in obstacles:
        vec = position - obstacle.center
        dist_to_center = np.linalg.norm(vec)
        d = dist_to_center - obstacle.radius  # расстояние до поверхности

        if d < d0 and d > 1e-6:  # препятствие оказывает влияние
            # Направление от препятствия к точке (радиальное)
            radial_dir = vec / (dist_to_center + 1e-10)

            # Величина радиальной силы (классическое отталкивание)
            radial_mag = k_rep * (1.0 / d - 1.0 / d0) * (1.0 / (d ** 2 + 1e-10))

            # Тангенциальное направление: поворот радиального вектора на +90° (против часовой стрелки)
            tangential_dir = np.array([-radial_dir[1], radial_dir[0]])

            # Величина тангенциальной (вихревой) силы пропорциональна радиальной
            tangential_mag = k_circ * radial_mag

            # Суммируем обе компоненты
            total_force += radial_mag * radial_dir + tangential_mag * tangential_dir

        elif d <= 0:  # робот внутри препятствия - сильное радиальное выталкивание
            if dist_to_center < 1e-6:
                direction = np.random.randn(2)
                direction /= np.linalg.norm(direction) + 1e-10
            else:
                direction = vec / (dist_to_center + 1e-10)
            total_force += k_rep * 10.0 * direction  # без вихря, только наружу

    return total_force


def total_force(position, goal, obstacles, k_att=1.0, k_rep=2.0, d0=2.0, k_circ=0.5):
    """Общая сила = притягивающая + отталкивающая с вихревой компонентой"""
    F_att = attractive_force(position, goal, k_att)
    F_rep = vortex_repulsive_force(position, obstacles, k_rep, d0, k_circ)
    return F_att + F_rep


def plan_path(start, goal, obstacles, step=0.05, max_iterations=5000,
              k_att=1.0, k_rep=2.0, d0=2.0, k_circ=0.5, goal_tolerance=0.1):
    """
    Планирование пути методом вихревых потенциальных полей.

    Параметры:
    - start: начальная точка [x, y]
    - goal: целевая точка [x, y]
    - obstacles: список препятствий
    - step: размер шага
    - max_iterations: максимальное количество итераций
    - k_att: коэффициент притяжения
    - k_rep: коэффициент радиального отталкивания
    - d0: радиус влияния препятствий
    - k_circ: коэффициент вихревой (циркуляционной) составляющей
    - goal_tolerance: допуск достижения цели
    """
    current = np.array(start, dtype=float)
    goal = np.array(goal, dtype=float)
    path = [current.copy()]
    prev_position = current.copy()
    stuck_counter = 0
    min_distance_to_goal = np.linalg.norm(current - goal)

    for i in range(max_iterations):
        # Вычисляем общую силу с вихревой компонентой
        force = total_force(current, goal, obstacles, k_att, k_rep, d0, k_circ)

        # Нормализуем силу и делаем шаг
        force_norm = np.linalg.norm(force)
        if force_norm > 1e-10:
            direction = force / force_norm
            new_position = current + step * direction
        else:
            # Если сила нулевая (редко), идём прямо к цели
            to_goal = goal - current
            to_goal_norm = np.linalg.norm(to_goal)
            if to_goal_norm > 1e-10:
                new_position = current + step * (to_goal / to_goal_norm)
            else:
                break  # уже у цели

        # Проверка на застревание (используется как страховка, но вихревое поле должно предотвращать)
        movement = np.linalg.norm(new_position - prev_position)
        distance_to_goal = np.linalg.norm(new_position - goal)

        if movement < 0.01 and distance_to_goal > goal_tolerance:
            stuck_counter += 1
        else:
            stuck_counter = 0
            min_distance_to_goal = min(min_distance_to_goal, distance_to_goal)

        # Если всё же застряли (например, в очень узком месте), применяем минимальную помощь
        if stuck_counter > 20:
            # Добавляем небольшое случайное возмущение
            random_dir = np.random.randn(2)
            random_dir /= np.linalg.norm(random_dir) + 1e-10
            new_position = current + step * 2.0 * random_dir
            stuck_counter = 0  # сброс

        # Обновляем текущую позицию
        current = new_position
        path.append(current.copy())
        prev_position = current.copy()

        # Проверка достижения цели
        if np.linalg.norm(current - goal) < goal_tolerance:
            return {
                'success': True,
                'path': np.array(path),
                'iterations': i + 1,
                'path_length': calculate_path_length(path)
            }

    return {
        'success': False,
        'path': np.array(path),
        'iterations': max_iterations,
        'path_length': calculate_path_length(path)
    }


def calculate_path_length(path):
    """Вычисление длины пути"""
    if len(path) < 2:
        return 0.0
    diffs = np.diff(path, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def visualize_path(path, obstacles, start, goal, title, save_path):
    """
    Визуализация пути и окружения
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Рисуем препятствия
    for obs in obstacles:
        circle = plt.Circle(obs.center, obs.radius, facecolor='gray',
                            alpha=0.6, edgecolor='black', linewidth=2)
        ax.add_patch(circle)

    # Рисуем путь
    if len(path) > 1:
        ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=2.5,
                label='Путь робота', alpha=0.8, zorder=3)
        # Точки пути (каждая 10-я)
        ax.scatter(path[::10, 0], path[::10, 1], c='blue',
                   s=20, alpha=0.5, zorder=4)

    # Начальная точка
    ax.scatter([start[0]], [start[1]], c='green', s=300,
               marker='o', label='Старт', edgecolors='black',
               linewidths=2, zorder=5)

    # Целевая точка
    ax.scatter([goal[0]], [goal[1]], c='red', s=400,
               marker='*', label='Цель', edgecolors='darkred',
               linewidths=2, zorder=5)

    # Определяем границы графика
    all_points = np.vstack([path, [start], [goal]])
    margin = 1.5
    x_min, y_min = all_points.min(axis=0) - margin
    x_max, y_max = all_points.max(axis=0) + margin

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('X координата', fontsize=12)
    ax.set_ylabel('Y координата', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold', pad=15)
    ax.legend(loc='upper right', fontsize=11, framealpha=0.9)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200, bbox_inches='tight')
    plt.close()


# ==================== ТЕСТОВЫЕ СЦЕНАРИИ ====================

def scenario_1_free_space():
    """Сценарий 1: Свободное пространство"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []
    return start, goal, obstacles


def scenario_2_single_obstacle():
    """Сценарий 2: Одно препятствие"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = [Obstacle([5.0, 5.0], 1.5)]
    return start, goal, obstacles


def scenario_3_two_obstacles():
    """Сценарий 3: Два препятствия"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = [
        Obstacle([4.0, 5.0], 1.2),
        Obstacle([6.0, 5.0], 1.2)
    ]
    return start, goal, obstacles


def scenario_4_narrow_corridor():
    """Сценарий 4: Узкий коридор"""
    start = np.array([0.0, 5.0])
    goal = np.array([10.0, 5.0])
    obstacles = []
    # Верхняя стена
    for x in np.linspace(1, 9, 9):
        obstacles.append(Obstacle([x, 7.0], 0.8))
    # Нижняя стена
    for x in np.linspace(1, 9, 9):
        obstacles.append(Obstacle([x, 3.0], 0.8))
    return start, goal, obstacles


def scenario_5_U_shaped_trap():
    """Сценарий 5: U-образная ловушка"""
    start = np.array([5.0, 2.0])
    goal = np.array([5.0, 9.0])
    obstacles = []

    # Нижняя стена
    for x in np.linspace(2, 8, 10):
        obstacles.append(Obstacle([x, 3.0], 0.7))

    # Левая стена
    for y in np.linspace(3, 9, 12):
        obstacles.append(Obstacle([2.0, y], 0.7))

    # Правая стена
    for y in np.linspace(3, 9, 12):
        obstacles.append(Obstacle([8.0, y], 0.7))

    return start, goal, obstacles

def scenario_6_obstacle_cluster():
    """
    Сценарий 6: Кластер из множества препятствий с узким извилистым проходом.
    Этот сценарий хорошо демонстрирует способность алгоритма с имитацией отжига
    и временным ослаблением полей находить путь в сложном окружении.
    """
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []

    # Центральный плотный кластер
    obstacles.append(Obstacle([4.0, 3.0], 1.2))
    obstacles.append(Obstacle([6.0, 3.5], 1.0))
    obstacles.append(Obstacle([3.5, 5.5], 1.0))
    obstacles.append(Obstacle([5.5, 5.0], 1.3))
    obstacles.append(Obstacle([4.5, 7.0], 1.1))
    obstacles.append(Obstacle([6.5, 6.5], 1.0))
    obstacles.append(Obstacle([7.0, 4.0], 0.9))
    obstacles.append(Obstacle([5.0, 4.0], 0.8))

    # Дополнительные препятствия по краям, чтобы направить поток
    obstacles.append(Obstacle([2.0, 1.0], 1.0))
    obstacles.append(Obstacle([8.0, 8.0], 1.0))

    return start, goal, obstacles


def scenario_7_dead_end_room():
    """
    Сценарий 7: Комната с узким выходом (тупик).
    Старт внутри комнаты, цель снаружи за стеной.
    Классическая ловушка для APF, требующая временного отключения
    отталкивающих сил или имитации отжига для нахождения выхода.
    """
    start = np.array([3.0, 3.0])
    goal = np.array([10.0, 3.0])
    obstacles = []

    # Параметры построения комнаты
    room_left = 1.0
    room_right = 5.0
    room_bottom = 0.0
    room_top = 6.0
    passage_y_min = 2.5
    passage_y_max = 3.5
    wall_radius = 0.5

    # Левая стена (сплошная)
    for y in np.linspace(room_bottom, room_top, 13):
        obstacles.append(Obstacle([room_left, y], wall_radius))

    # Верхняя стена
    for x in np.linspace(room_left, room_right, 9):
        obstacles.append(Obstacle([x, room_top], wall_radius))

    # Нижняя стена
    for x in np.linspace(room_left, room_right, 9):
        obstacles.append(Obstacle([x, room_bottom], wall_radius))

    # Правая стена: верхняя часть
    for y in np.linspace(passage_y_max, room_top, 5):
        obstacles.append(Obstacle([room_right, y], wall_radius))

    # Правая стена: нижняя часть
    for y in np.linspace(room_bottom, passage_y_min, 5):
        obstacles.append(Obstacle([room_right, y], wall_radius))

    # Внутреннее препятствие, усложняющее движение
    obstacles.append(Obstacle([2.5, 3.0], 0.8))

    # Дополнительные препятствия снаружи, чтобы направить к цели
    obstacles.append(Obstacle([7.0, 2.0], 0.7))
    obstacles.append(Obstacle([8.0, 4.0], 0.7))

    return start, goal, obstacles

# ==================== ГЛАВНАЯ ФУНКЦИЯ ====================

def run_experiments():
    """
    Запуск всех экспериментов с сохранением результатов
    """
    # Создаем папку для результатов с временем запуска
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_dir = f"results_vortex_{timestamp}"
    os.makedirs(results_dir, exist_ok=True)

    print("=" * 70)
    print("ЭКСПЕРИМЕНТЫ ПО АЛГОРИТМУ ВИХРЕВЫХ ПОЛЕЙ (VORTEX FIELDS)")
    print("=" * 70)
    print(f"Результаты будут сохранены в: {results_dir}")
    print("=" * 70 + "\n")

    # Определяем сценарии
    scenarios = [
        ("1_Свободное_пространство", scenario_1_free_space),
        ("2_Одно_препятствие", scenario_2_single_obstacle),
        ("3_Два_препятствия", scenario_3_two_obstacles),
        ("4_Узкий_коридор", scenario_4_narrow_corridor),
        ("5_U_образная_ловушка", scenario_5_U_shaped_trap),
        ("6_Кластер_препятствий", scenario_6_obstacle_cluster),
        ("7_Комната_с_узким_выходом", scenario_7_dead_end_room),
    ]

    # Параметры алгоритма (с добавлением k_circ)
    params = {
        'step': 0.08,
        'max_iterations': 5000,
        'k_att': 1.5,
        'k_rep': 2.5,
        'd0': 2.5,
        'k_circ': 0.8,  # коэффициент вихревой составляющей
        'goal_tolerance': 0.15
    }

    # Сохраняем параметры в файл
    params_file = os.path.join(results_dir, "parameters.txt")
    with open(params_file, 'w', encoding='utf-8') as f:
        f.write("Параметры алгоритма вихревых полей\n")
        f.write("=" * 50 + "\n")
        for key, value in params.items():
            f.write(f"{key}: {value}\n")
        f.write(f"\nВремя запуска: {timestamp}\n")

    # Запускаем эксперименты
    results_summary = []

    for scenario_name, scenario_func in scenarios:
        print(f"\nОбработка сценария: {scenario_name}")
        start, goal, obstacles = scenario_func()

        # Планируем путь
        result = plan_path(start, goal, obstacles, **params)

        # Формируем название для сохранения
        title = f"Сценарий: {scenario_name} (вихревые поля)\n"
        title += f"Успех: {'Да' if result['success'] else 'Нет'}, "
        title += f"Длина пути: {result['path_length']:.2f}, "
        title += f"Итераций: {result['iterations']}"

        # Сохраняем визуализацию
        image_path = os.path.join(results_dir, f"{scenario_name}.png")
        visualize_path(result['path'], obstacles, start, goal, title, image_path)

        # Сохраняем информацию о результате
        results_summary.append({
            'scenario': scenario_name,
            'success': result['success'],
            'path_length': result['path_length'],
            'iterations': result['iterations']
        })

        print(f"  [OK] Успех: {result['success']}")
        print(f"  [OK] Длина пути: {result['path_length']:.2f}")
        print(f"  [OK] Итераций: {result['iterations']}")
        print(f"  [OK] Изображение сохранено: {image_path}")

    # Сохраняем сводку результатов
    summary_file = os.path.join(results_dir, "results_summary.txt")
    with open(summary_file, 'w', encoding='utf-8') as f:
        f.write("СВОДКА РЕЗУЛЬТАТОВ ЭКСПЕРИМЕНТОВ (ВИХРЕВЫЕ ПОЛЯ)\n")
        f.write("=" * 70 + "\n\n")
        for res in results_summary:
            f.write(f"Сценарий: {res['scenario']}\n")
            f.write(f"  Успех: {'Да' if res['success'] else 'Нет'}\n")
            f.write(f"  Длина пути: {res['path_length']:.2f}\n")
            f.write(f"  Итераций: {res['iterations']}\n")
            f.write("-" * 70 + "\n")

    print("\n" + "=" * 70)
    print("ВСЕ ЭКСПЕРИМЕНТЫ ЗАВЕРШЕНЫ!")
    print(f"Результаты сохранены в папку: {results_dir}")
    print("=" * 70)

    return results_summary


if __name__ == "__main__":
    run_experiments()