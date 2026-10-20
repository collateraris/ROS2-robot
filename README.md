# Room robot v5: четыре недели, RPLIDAR C1 и RF2O без энкодеров

Проект переработан под Ubuntu 24.04, ROS 2 Jazzy и Gazebo Harmonic. На настоящем роботе колёсных энкодеров нет: перемещение оценивает `rf2o_laser_odometry` по сканам C1. Arduino получает номинальную скорость через калиброванный PWM-мост; PWM не служит одометрией.

**Начать с [установки среды](docs/INSTALL_RU.md), затем выполнять недели последовательно.** Это исходники и инструкции для проверки на вашем стенде. Готовой карты вашей комнаты, калибровки шасси и результатов поездок в поставке нет.

Если на ПК уже Ubuntu: [подробная первая неделя на Ubuntu-ПК](docs/WEEK01_UBUNTU_RU.md), от проверки версии ОС до A → B и сохранения карты.

Если на ПК Windows: [подробная первая неделя с Ubuntu 24.04 в WSL2](docs/WEEK01_WINDOWS_RU.md). Симуляция запускается на ПК, отдельная проверка настоящего C1 — на Raspberry Pi. Обновление 19.10.2026 исправляет порядок ARM в примере A → B: он запрашивается отдельно для каждой принятой цели после появления свежей команды контроллера, без повторного ARM внутри прерванного движения.

## Код по неделям

| Неделя | Реализованный результат | Инструкция и пояснения к коду | Самостоятельный накопительный срез |
|---|---|---|---|
| 1 | Gazebo: четырёхколёсный робот, LiDAR/RGB, RF2O, SLAM/AMCL, Nav2, A → B | [WEEK01_RU.md](docs/WEEK01_RU.md), [индекс исходников](weeks/week01/SOURCE_INDEX.md) | [week01.zip](weekly_releases/week01.zip) |
| 2 | Настоящий Grounding DINO, camera–LiDAR привязка, ID/TTL, оценка неопределённости | [WEEK02_RU.md](docs/WEEK02_RU.md), [индекс исходников](weeks/week02/SOURCE_INDEX.md) | [week02.zip](weekly_releases/week02.zip) |
| 3 | Rules baseline и локальный LLM/VLM через Ollama, структурный план, проверка пути, Nav2 | [WEEK03_RU.md](docs/WEEK03_RU.md), [индекс исходников](weeks/week03/SOURCE_INDEX.md) | [week03.zip](weekly_releases/week03.zip) |
| 4 | Train/dev/test сцены, симуляционный runner, журналы, CSV/JSON и график метрик | [WEEK04_RU.md](docs/WEEK04_RU.md), [индекс исходников](weeks/week04/SOURCE_INDEX.md) | [week04.zip](weekly_releases/week04.zip) |

Главный каталог содержит весь проект. Недельные архивы — отдельные срезы с кодом недель 1..N: распаковывать каждый в собственную папку. Для работы достаточно одного полного проекта; смешивать файлы разных срезов не нужно. [План по дням и критерии приёмки](weeks/README.md).

## Что изменено в исходном плане

- Gazebo Harmonic выбран для единой связки с Jazzy, Nav2 и ROS-сенсорами. MuJoCo/Isaac Sim в этой поставке не поддерживаются.
- LLM выбирает `goto_object`, `goto_waypoint`, `wait`, `stop` из известных ID/имён. Метрическую цель вычисляет код по измерениям, затем проверяет карту и весь путь. Модель не генерирует произвольные координаты.
- Камера и плоский LiDAR дают приближённую позицию отражающего кластера в плоскости сканирования. Это не 3D-центр объекта. Нужны проверенные intrinsics/extrinsics и согласованные timestamps.
- Tiny-VLA и world model сохранены как отдельные offline/shadow эксперименты. Они не заменяют Nav2 и не являются универсальной предобученной VLA.
- Четыре недели — этапы с проверяемыми результатами. Срок зависит от оборудования, калибровки и доступного времени.

## Установка и первый запуск

После установки ROS 2 Jazzy по [INSTALL_RU.md](docs/INSTALL_RU.md), в свежем терминале без активного venv/Conda:

```bash
cd ~/robot/room_robot_v5_four_weeks
bash scripts/install_system.sh sim
source /opt/ros/jazzy/setup.bash
# Один раз, если rosdep ещё не настроен: sudo rosdep init
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro jazzy
bash scripts/build_ros.sh all
bash scripts/setup_python.sh base
source scripts/env_robot.sh
ros2 launch room_sim sim.launch.py start_mission:=false
```

В каждом следующем терминале сначала `cd` в корень и `source scripts/env_robot.sh`. Сборку colcon выполнять до активации venv. Для детектора/обучения установить подходящий PyTorch, затем `bash scripts/setup_python.sh all existing`; CPU-проверка: `bash scripts/setup_python.sh all cpu`. CPU не гарантирует допустимую задержку live-восприятия.

После проверки `/scan`, `/odom`, TF и карты выполнить A → B строго по инструкции недели 1. Автоматический ARM разрешён только отдельным симуляционным runner. Обычный запуск оставляет привод disarmed.

## Рабочая цепочка

`C1 → /scan → RF2O → odom_guard → /odom → SLAM Toolbox/AMCL + Nav2`.

`RGB + /scan + calibration + TF → metric objects → rules/LLM plan → task coordinator → Nav2`.

`Nav2 → velocity smoother → Collision Monitor → /cmd_vel_safe → MCU PWM-мост` на железе либо изолированный `/sim/cmd_vel` в Gazebo. Независимые ARM, deadman, STOP и watchdog остаются в цепочке. Истинная поза симулятора `/sim/odom` доступна только оценщику и не исправляет рабочую одометрию.

Основные каталоги:

| Каталог | Ответственность |
|---|---|
| `ros2_ws/src/room_base`, `room_nav` | Общая навигация, guard, настоящий encoderless привод |
| `ros2_ws/src/room_sim` | Модель/мир, bridge, адаптер симуляционного привода, A → B |
| `room_semantics`, `ros2_ws/src/room_perception` | Детектор, геометрия, tracking и ROS-узел восприятия |
| `room_planning`, `ros2_ws/src/room_tasks` | Строгая схема плана, backend, проверка карты/пути, action clients |
| `room_evaluation`, `ros2_ws/src/room_eval` | Сценарии, GT-оценка, отчёты; GT не передаётся планировщику |
| `learning_nav` | Отдельные world model / finite-waypoint tiny-VLA, схема данных 3 |
| `firmware/room_base_rf2o` | Активная прошивка V5 для Uno/classic Nano, CRC/session/watchdog |

`legacy_v2` и `firmware/room_base_v3` — исторические исходники, не активная цепочка управления.

## Настоящий RPLIDAR C1, SLAM и шасси

[Полная установка и USB-порты](docs/INSTALL_RU.md), [C1, RF2O, SLAM и сохранение карты](docs/NAVIGATION_RU.md), [пины Arduino, калибровка PWM и STOP](docs/BASE_HARDWARE_RU.md). Измерить габариты, положение сенсоров и кривые скорости своего робота. Примерные числа из YAML не являются калибровкой.

Первую карту строить с `start_bridge:=false` и отключённым питанием моторов. C1 использует 460800 бод, Arduino — 115200. Использовать устойчивые `/dev/serial/by-id/` или `/dev/serial/by-path/`. Энкодерный PI не запускается. На реальном шасси управление по умолчанию выключено до подтверждения калибровки; подтверждение не выполняет ARM.

## Проверка и границы результата

```bash
python scripts/doctor.py --ros --sim
bash scripts/verify_offline.sh
```

Выполненные проверки и ограничения: [VALIDATION_RU.md](docs/VALIDATION_RU.md). Есть настоящий одиночный inference предобученного Grounding DINO; его CPU-задержка около 9.17 с не подходит для заданных live freshness limits. ROS/Gazebo, Ollama/Qwen и настоящее железо в среде подготовки не запускались. Синтетические smoke-метрики явно помечены и не доказывают автономность.

Геометрический манекен в базовом мире — proxy для геометрии/контактов, а не доказательство person detection. Для исследования zero-shot нужны подходящие изображения/реалистичные сцены, измеренные задержки и реальные журналы испытаний. Оценщик выдаёт `incomplete`, если отсутствует наблюдаемость контактов, сканов или GT.

[Архитектура и контракты](docs/ARCHITECTURE_RU.md). [Запись данных и обучение](docs/LEARNING_RU.md). [Версии RF2O/Slamtec и локальные патчи](vendor/UPSTREAM.md).
