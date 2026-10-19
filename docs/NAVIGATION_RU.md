# C1 → RF2O → SLAM/AMCL → Nav2

## 1. Сборка и порты

Следуй установке в README. В архиве исходники драйверов; `fetch_ros_drivers.sh` проверяет фиксированные версии и при необходимости восстанавливает отсутствующие пакеты. Сборка включает room_base, room_nav, RF2O и sllidar_ros2. C1 использует serial baud **460800**, драйвер `sllidar_node`, scan mode `Standard`. Arduino использует другой порт и baud **115200**.

```bash
ls -l /dev/serial/by-id/
source scripts/env_robot.sh
```

Запускать один экземпляр каждого узла. Если LiDAR/RF2O запущены вручную, передать `start_lidar:=false` / `start_rf2o:=false`; проверить темы и TF. Не запускать штатный Nav2 bringup параллельно.

## 2. Карта без моторного привода

Отключи питание моторов. Закрепи C1 горизонтально. `robot_radius` должен охватывать весь корпус. Измерь смещение C1, включая знак осей: x вперёд, y влево, z вверх. В следующей команде геометрия примерная:

```bash
ros2 launch room_nav bringup.launch.py mode:=mapping start_bridge:=false \
  lidar_port:=/dev/serial/by-id/REPLACE_WITH_C1 \
  robot_radius:=0.18 laser_x:=0.0 laser_y:=0.0 laser_z:=0.15
```

По умолчанию launch публикует static TF `base_link → laser`; если URDF уже публикует его, задать `publish_laser_tf:=false`. Дубли TF недопустимы. Колёсная одометрия и identity TF не нужны. Для RF2O нужен настоящий static TF даже если лазер расположен по центру.

```bash
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /odom
ros2 topic echo /room_base/odom_status
ros2 run tf2_ros tf2_echo odom base_link
rviz2
```

Медленно двигай робот вручную по полу, не поднимай/наклоняй. Начни со сцены с несколькими неподвижными стенами/предметами. C1 обычно выдаёт около 10 сканов/с; `/odom_rf2o` и `/odom` должны обновляться по новым сканам, а не выдавать старые оценки с новым временем. Неподвижный робот должен оставаться примерно неподвижным; направление метров/поворотов сверить вручную.

Сохранение occupancy map для AMCL:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/room"
```

## 3. Привод

Прошивка и калибровка описаны в BASE_HARDWARE_RU.md. Включение serial и ARM раздельны. При `start_bridge:=false` моторной связи нет. При `start_bridge:=true` мост остаётся disarmed до выполнения условий и запроса оператора.

```bash
ros2 launch room_nav bringup.launch.py mode:=mapping \
  lidar_port:=/dev/serial/by-id/REPLACE_WITH_C1 \
  base_config:="$PWD/config/robot_base.yaml" \
  robot_radius:=0.18 laser_x:=0.0 laser_y:=0.0 laser_z:=0.15
```

Затем в другом терминале, из корня: `python -m robot_extras.operator`. В RViz задавать Nav2 Goal. Удерживать кнопку, **A** — ARM; **Esc** — STOP. Потеря deadman, скана, одометрии или ACK снимает ARM. Обычная команда нулевой скорости просто выключает PWM в существующей сессии, если остальные условия действуют.

## 4. Сохранённая карта

Остановить mapping launch. Запускать с той же установкой LiDAR и калибровкой:

```bash
ros2 launch room_nav bringup.launch.py mode:=localization \
  map:="$PWD/maps/room.yaml" \
  lidar_port:=/dev/serial/by-id/REPLACE_WITH_C1 \
  base_config:="$PWD/config/robot_base.yaml" \
  robot_radius:=0.18 laser_x:=0.0 laser_y:=0.0 laser_z:=0.15
```

RF2O начинает новую локальную систему `odom`; AMCL связывает её с картой. В RViz поставить **2D Pose Estimate**, проверить совпадение скана с картой, затем Nav2 Goal. Если RF2O/guard фиксируют отказ, сначала STOP, остановить весь launch, затем заново локализоваться. Перезапускать только RF2O с прежним TF/map состоянием нельзя.

## 5. Текстовые цели

Заменить примерные координаты в `room_nav/config/waypoints.yaml`, включить `enable_waypoints:=true` и передать `waypoints:=/absolute/path/waypoints.yaml`. Примерные точки отключены по умолчанию.

```bash
ros2 topic pub --once /room_nav/text_command std_msgs/msg/String "{data: 'к столу'}"
ros2 topic pub --once /room_nav/text_command std_msgs/msg/String "{data: 'стоп'}"
```

Неограниченное exploration неизвестной комнаты не добавлено. Для первой карты достаточно ручного перемещения.

## 6. Диагностика

| Наблюдение | Проверить |
|---|---|
| Нет `/scan` | C1 USB/питание, 460800, права dialout, правильный отдельный порт |
| Есть `/scan`, нет `/odom_rf2o` | static TF, качество отражений, лог RF2O, соответствие bundled версии |
| Нет `/odom` или health false | `/room_base/odom_status`, acquisition stamps, отсутствие скачков, распределение отражений |
| Карта едет боком/вращается | лазерные оси и yaw, размещение C1, скорость ручного перемещения |
| Отказ ARM | calibration_confirmed/allow_actuation, порт MCU, V5, deadman, свежие команды/health/ACK |
| Нет медленного вращения | минимальная непрерывная скорость моторов против угловых лимитов Nav2 |
| Отказ `fault latched` | STOP и перезапуск всей навигационной сессии; устранить причину |

Свежие сообщения и health не доказывают правильность позы. Проверка реальных расстояний, поворотов и торможения остаётся частью запуска на роботе.
