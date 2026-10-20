# Неделя 1 на Ubuntu-ПК: установка → Gazebo → RF2O → SLAM → Nav2

Обновлено: 20.10.2026.

На ПК уже стоит Ubuntu. Основной маршрут ниже выполняется **локально на ПК**,
без WSL. Он рассчитан на Ubuntu 24.04 Noble, ROS 2 Jazzy, Gazebo Harmonic.
Сначала проверяем версию ОС, затем устанавливаем совместимые пакеты.

Результат недели — виртуальный четырёхколёсный робот получает одометрию из
лидара через RF2O, строит карту, едет к двум заданным точкам через Nav2 и
останавливается по команде. Энкодеры не нужны. Настоящий C1 можно проверить
отдельно на ПК либо на Raspberry Pi; инструкции находятся в разделе 10.

Реальная моторная база, её калибровка, ML-детектор и LLM относятся к следующим
этапам. Для первой недели достаточно базового Python-окружения проекта.

## 1. Где выполнять команды

| Терминал | Как открыть | Назначение |
|---|---|---|
| A: локальный ПК | Ctrl+Alt+T | Установка/сборка, затем общий launch симуляции |
| B: локальный ПК | Ещё один терминал | Топики, TF, здоровье, STOP, сохранение карты |
| C: локальный ПК | Ещё один терминал | Сценарий A → B |
| SSH Pi, необязательно | `ssh shapturov@192.168.0.20` | Только независимая проверка C1 на Pi |

Проверить машину можно командой `hostname`. Симуляцию и графические окна
запускай на ПК. В основные блоки ниже **не входи по SSH на Pi**.
При вводе Linux-пароля символы и звёздочки не отображаются — это нормально.

Для симуляции используем `ROS_DOMAIN_ID=42`; для отдельной проверки физического
C1 — `43`. Это независимые графы: настоящий и виртуальный `/scan` не смешиваются.
Все терминалы одного испытания должны использовать одинаковый domain.

## 2. Проверить установленную Ubuntu

В локальном терминале ПК:

```bash
cat /etc/os-release
/usr/bin/python3 --version
uname -m
```

Для следующих команд ожидаются `VERSION_ID="24.04"`, `VERSION_CODENAME=noble`
и системный Python 3.12. На обычном Intel/AMD-ПК архитектура — `x86_64`.
Если версия Ubuntu другая, не подключай к ней Noble-репозиторий из следующего
раздела. Сначала нужен согласованный вариант ОС и ROS; смешивание пакетов разных
Ubuntu не исправляется заменой системного Python.

Если Jazzy уже установлен и `source /opt/ros/jazzy/setup.bash` работает,
повторно добавлять ROS-репозиторий не нужно: пропусти только блок его
подключения в разделе 3. Общую установку пакетов перед проверкой графики
выполни, даже если ранее стоял только `ros-base`.

## 3. Установить ROS 2 Jazzy и проверить графику

Все команды этого раздела — **в локальной Ubuntu на ПК**, обычным пользователем, с `sudo`
только там, где оно показано. Не используй Conda и не активируй Python venv.

```bash
sudo apt update
sudo apt install -y curl python3 software-properties-common locales
sudo add-apt-repository -y universe
locale
```

В выводе locale должна использоваться кодировка UTF-8. Если её нет:

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Подключи официальный ROS apt-репозиторий. Используется пакет
`ros2-apt-source`; URL ниже предназначен для проверенного `noble`:

```bash
room_ros_repo_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')

curl -fL \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${room_ros_repo_version}/ros2-apt-source_${room_ros_repo_version}.noble_all.deb" \
  -o /tmp/room-ros2-apt-source.deb

sudo dpkg -i /tmp/room-ros2-apt-source.deb
```

Общая установка для свежего и ранее установленного Jazzy:

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz mesa-utils pciutils unzip xdg-user-dirs
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Проверка OpenGL:

```bash
lspci -nn | grep -Ei 'VGA|3D|Display'
echo "$XDG_SESSION_TYPE"
glxinfo -B
rviz2
```

Должно открыться окно RViz на рабочем столе Ubuntu. Закрой его перед следующим
этапом. В `glxinfo -B` посмотри renderer: `llvmpipe` означает программный
рендеринг, который может быть слишком медленным для сенсоров этого мира.

Если установлена NVIDIA, отдельно проверь `nvidia-smi`. Для Intel/AMD эта
команда не нужна. Успешный `nvidia-smi` не заменяет проверку OpenGL. Не меняй
драйвер вслепую и не устанавливай CUDA Toolkit ради первой недели: сначала
проверь существующий renderer и конкретный вывод ошибки.

ROS Jazzy ставится на Ubuntu 24.04 по [официальной инструкции ROS](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
При конфликтах установки `ros-dev-tools` проверь наличие `noble-updates` и
`noble-backports` в `/etc/apt/sources.list.d/ubuntu.sources`, как описано там.

## 4. Скачать проект и поставить системные зависимости

Скачай актуальный `room_robot_v5_four_weeks.zip` через браузер Ubuntu.
Команда `xdg-user-dir DOWNLOAD` находит папку загрузок независимо от языка
рабочего стола. В локальном терминале:

```bash
room_download_dir=$(xdg-user-dir DOWNLOAD)
ls -lh "$room_download_dir/room_robot_v5_four_weeks.zip"
mkdir -p ~/robot
unzip "$room_download_dir/room_robot_v5_four_weeks.zip" -d ~/robot
cd ~/robot/room_robot_v5_four_weeks
bash scripts/install_system.sh sim
```

Если браузер переименовал ZIP или сохранил его в другую папку, используй
фактический путь к этому файлу. Для повторной установки сначала сохрани свои
карты/конфигурации; не распаковывай новую поставку поверх них вслепую.
Далее инструкция предполагает корень `~/robot/room_robot_v5_four_weeks`.

`install_system.sh sim` устанавливает Nav2, SLAM Toolbox, инструменты сборки
и `ros-jazzy-ros-gz`. Официальная пара для Jazzy — Gazebo Harmonic из ROS apt.
Отдельно устанавливать Gazebo Classic или подключать OSRF-репозиторий для
этого маршрута не требуется.

До сборки проверь базовый Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
gz sim shapes.sdf -v 4
```

Откроется стандартный мир с фигурами. Закрой его полностью перед проектным
launch. Это проверка GUI и базового мира; датчики проекта проверяются ниже.
При конкретной ошибке Wayland/currentGLContext смотри раздел 11.

## 5. Собрать только пакеты первой недели

Открой **новый терминал Ubuntu без venv/Conda**. Сначала ROS-сборка,
только потом Python-окружение:

```bash
cd ~/robot/room_robot_v5_four_weeks
source /opt/ros/jazzy/setup.bash
bash scripts/fetch_ros_drivers.sh
```

Исходники драйверов уже включены; скрипт проверяет подготовленные версии и при
необходимости загружает отсутствующие. Если rosdep ещё ни разу не настраивался,
один раз выполни:

```bash
sudo rosdep init
```

Если команда пишет, что sources list уже существует, повторная инициализация
не нужна. Обновление запускается обычным пользователем:

```bash
rosdep update
rosdep install --from-paths \
  ros2_ws/src/room_base \
  ros2_ws/src/room_nav \
  ros2_ws/src/room_sim \
  ros2_ws/src/rf2o_laser_odometry \
  ros2_ws/src/sllidar_ros2 \
  --ignore-src -r -y --rosdistro jazzy

bash scripts/build_ros.sh week01
bash scripts/setup_python.sh base
source scripts/env_robot.sh
export ROS_DOMAIN_ID=42
```

`week01` собирает базу, навигацию, симулятор и нужные драйверы. Весь поздний
ML-стек не устанавливается. `setup_python.sh base` создаёт `.venv` системным
Python 3.12 с доступом к ROS-пакетам и ставит базовые Python-зависимости.

Проверки:

```bash
python -c 'import rclpy, room_base.core; print("ROS Python OK")'
ros2 pkg prefix room_sim
ros2 pkg prefix room_nav
ros2 pkg prefix rf2o_laser_odometry
```

Успешный `colcon build` заканчивается Summary без Failed/Aborted packages.
При ошибке сохрани вывод с именем первого упавшего пакета и его ошибками;
не запускай симуляцию поверх незавершённой сборки.

### Каждый следующий терминал на ПК

Открой новое локальное окно терминала на ПК. В каждом окне выполни:

```bash
cd ~/robot/room_robot_v5_four_weeks
source scripts/env_robot.sh
export ROS_DOMAIN_ID=42
```

Это обязательный префикс для всех следующих терминалов ПК. Скрипт подключает
ROS и colcon overlay, активирует venv и добавляет исходники в `PYTHONPATH`.
Он должен выполняться через **source**, не `bash`.

Необязательный отдельный тест ROS: в двух подготовленных терминалах ПК
`ros2 run demo_nodes_cpp talker` и `ros2 run demo_nodes_py listener`. Если listener
получает сообщения, базовый локальный DDS работает. Останови оба через Ctrl+C.
Это ещё не проверка навигации или связи с Raspberry Pi.

## 6. Первый запуск: сенсоры и карта, без движения

В терминале A, после обязательного префикса:

```bash
ros2 launch room_sim sim.launch.py start_mission:=false
```

Откроются Gazebo и RViz. В launch уже включены:

| Компонент | Его работа |
|---|---|
| Gazebo + `ros_gz_bridge` | Физика, виртуальные LiDAR/камера, передача сенсоров в ROS |
| `robot_state_publisher` | Постоянная геометрия сенсоров относительно корпуса |
| RF2O | Перемещение по последовательным лазерным сканам |
| odom guard | Проверка свежести/правдоподобия и публикация рабочей `/odom` |
| SLAM Toolbox | Карта `/map`, поправка `map → odom` |
| Nav2 + Collision Monitor | Планирование и проверка команд перед приводом |
| Simulation drive adapter | Команды только виртуальному приводу, ARM и watchdog |

`start_mission:=false` отключает прежний узел именованных задач. Настоящий C1,
Arduino serial bridge и реальные моторы этот launch не запускает. По умолчанию
`simulation_autonomy=false`, а привод остаётся disarmed.

### Проверки из терминала B

После обязательного префикса выполняй команды **по одной**. `topic hz`,
`topic echo` без `--once` и `tf2_echo` продолжаются до Ctrl+C.

```bash
ros2 topic echo /clock --once
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /room_base/odom_healthy --once
ros2 topic echo /room_base/odom_status --once
ros2 topic hz /odom_rf2o
ros2 topic hz /odom
```

Ожидаемая номинальная частота LiDAR — 10 Гц, камеры — 15 Гц. `ros2 topic hz`
измеряет фактическую частоту доставки: при медленной симуляции она может быть
ниже. Важны ещё продвижение `/clock`, свежие stamps и отсутствие fault.
Значение `/room_base/odom_healthy` должно стать `true` после нескольких
согласованных сканов. `/odom_rf2o` и `/odom` должны обновляться по новым сканам,
а не по таймеру, повторяющему старую позу с новым временем.

Проверка TF, тоже по одной команде:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo map base_link
```

Нужна связная геометрия `map → odom → base_link`; от `base_link` идут ветви
к `laser` и `camera_link → camera_optical_frame`. Не добавляй собственные
identity TF и не подключай `/sim/odom` вместо RF2O: истинная поза симулятора
предназначена только для отдельной оценки ошибок.

### Что увидеть в RViz

- Global Options → Fixed Frame: `map`.
- Map: `/map`, durability `Transient Local`.
- LaserScan: `/scan`, reliability `Best Effort`.
- Image: `/camera/image_raw`, reliability `Best Effort`.
- TF: положения корпуса и сенсоров связаны с картой.

Проектный профиль уже содержит эти displays. Если карта ещё не появилась,
дай SLAM несколько сканов и один цикл обновления карты; затем проверяй логи
и TF. Без действующей `/odom` карту исправлять бессмысленно.

Один видимый мир Gazebo и даже окно RViz не подтверждают работоспособность
сенсорного контура. Критерий — реальные сообщения `/scan`, изображения,
здоровая RF2O-одометрия и согласованная карта. Манекен человека в комнате —
геометрический proxy; распознавание людей на этой неделе не проверяется.

## 7. Проехать к A и B только в симуляции

Останови launch в терминале A через Ctrl+C. Дождись выхода его процессов и
закрытия мира; не запускай вторую копию параллельно. Подними новую сессию:

```bash
ros2 launch room_sim sim.launch.py \
  start_mission:=false simulation_autonomy:=true
```

`simulation_autonomy=true` разрешает отдельному симуляционному сценарию работать
без удерживаемой операторской кнопки. Сам этот параметр **не вызывает ARM**.
Он относится только к `room_sim_drive`, не к аппаратной базе.

В терминале B снова проверь `/clock`, `/room_base/odom_healthy`, карту и TF.
Для готовности Nav2:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list -t
```

Ожидаются active у навигационных узлов и действие
`/navigate_to_pose` типа `nav2_msgs/action/NavigateToPose`.

В терминале C, после обязательного префикса:

```bash
ros2 run room_sim goal_ab --arm-simulation \
  --goal 0.6 0.0 0.0 \
  --goal 1.4 0.0 0.0 \
  --timeout 90
```

Это **метры и yaw в радианах в системе `map`**, а не PWM и не координаты пикселя.
Для исходного мира старт близок к `(0,0,0)`; A и B лежат впереди в свободной
полосе. Сначала убедись по построенной карте, что они свободны. После изменения
мира, старта или загруженной карты подставлять эти числа автоматически нельзя.

Скрипт отправляет настоящие `NavigateToPose` последовательно. После принятия
каждой новой цели и проверки симуляционного контура он делает не более одного
ARM для этой цели; ожидание планирования проходит с выключенным приводом.
Переход к B возможен только после успешного завершения A. Это не повторный ARM
старой поездки после отказа: при STOP/ошибке текущий сценарий прекращается.
В конце он останавливает привод и освобождает собственные навигационные цели.

В терминале должны появиться сообщения о достижении A, затем B. В RViz скан,
положение робота и карта должны оставаться согласованными; физическая модель
должна двигаться без заметных перескоков и пересечений с мебелью.

Здесь цели задаются именно `goal_ab` через action. Кнопка обычного RViz
`2D Goal Pose` лишь публикует `/goal_pose`; она не является в этой инструкции
заменой проверенному action-клиенту и процедуре ARM.

## 8. STOP, прерывание и повторный запуск

В терминале B можно остановить виртуальный привод:

```bash
ros2 service call /room_base/stop std_srvs/srv/Trigger '{}'
```

Наблюдение остановки:

```bash
ros2 topic echo /sim/cmd_vel --once
ros2 topic echo /room_base/armed --once
ros2 topic echo /room_base/status --once
```

Ожидается нулевой выход и снятый ARM. Сам STOP привода не доказывает, что action
Nav2 уже отменён. В терминале C нажми Ctrl+C, дай `goal_ab` завершить отмену
принадлежащих ему целей и STOP. Не запускай повторный ARM, пока неизвестен
результат отмены предыдущей цели. Если лог сообщает об unconfirmed cancellation,
полностью перезапусти launch/Nav2 перед следующей поездкой.

При потере сенсоров, fault RF2O/guard, существенном торможении рендера либо
защёлкнутом отказе используй понятную процедуру:

1. Выполни STOP и прерви `goal_ab`; дай ему завершить cleanup.
2. Останови общий launch через Ctrl+C и дождись выхода Gazebo и ROS-процессов.
3. Устрани причину: графика/нагрузка/clock/конфигурация.
4. Запусти весь launch заново, проверь сенсоры, health, карту и TF.
5. Для localization заново задай начальную pose. Только затем отправляй новую
   явную задачу.

Не нажимай Gazebo Reset посреди работающего ROS-стека: скачок времени и позы
может нарушить одометрию. Пауза Gazebo также останавливает сенсоры и может
привести к срабатыванию watchdog; не рассчитывай на автоматическое продолжение.

У адаптера есть `/room_base/reset_estop`, но его успешный ответ только снимает
допустимую защёлку в остановленном состоянии — он не отменяет чужие action,
не исправляет RF2O и не выдаёт ARM. Для первого знакомства после неизвестного
состояния используй полный перезапуск выше, а не цикл reset/rearm.

## 9. Сохранить карту и проверить AMCL

Сохраняй карту, пока mapping launch ещё работает и робот уже остановлен.
В подготовленном терминале B:

```bash
mkdir -p data/maps
ros2 run nav2_map_server map_saver_cli \
  -f "$ROOM_PROJECT_ROOT/data/maps/room" \
  --ros-args -p use_sim_time:=true
ls -lh data/maps/room*
```

Ожидается `room.yaml` и файл изображения карты, на который он ссылается
(например, `room.pgm`). Один YAML без изображения недостаточен. Это occupancy
map для AMCL, а не сериализованный pose graph для продолжения SLAM.

Останови mapping launch и запусти свежий мир с сохранённой картой:

```bash
ros2 launch room_sim sim.launch.py \
  mode:=localization \
  map:="$ROOM_PROJECT_ROOT/data/maps/room.yaml" \
  start_mission:=false simulation_autonomy:=true
```

Теперь карту публикует map_server, положение на ней уточняет AMCL. RF2O начинает
новую локальную `odom`; AMCL должен связать её с сохранённой `map`.

В RViz выбери **2D Pose Estimate** и поставь начальную позу робота. Нажми в
правильной точке карты и протяни стрелку в направлении его взгляда. Для свежего
запуска исходного `room.sdf` физическая модель снова появляется на старте;
это примерно `(0,0)` карты первой mapping-сессии, со взглядом вдоль `+X`
(yaw `0`). **Не ставь её в B `(1.4,0)` только потому, что предыдущая поездка
закончилась там:** перезапуск мира вернул физическую модель на старт.

Стрелка обозначает ориентацию робота, а не направление будущей поездки.
Если ты менял старт/мир/карту, используй реальное соответствие этих систем,
а не этот частный пример. `origin` в map YAML — положение изображения карты,
оно не является автоматически initial pose робота.

Проверь совпадение лазерных стен с картой, `map → base_link`, health и active
Nav2. Только после этого повтори `goal_ab`. Если локализация не совпала,
исправь initial pose; не подменяй её скрытой `/sim/odom` в рабочей навигации.

## 10. Необязательно: проверить настоящий C1 отдельно

Этот раздел проверяет только физический C1 по USB, без управления колёсами.
Выбери место подключения: Ubuntu-ПК (вариант A) или Pi (вариант B).
Для проверки скана запускается только `sllidar_ros2`, моторный мост не нужен.

### Вариант A: C1 подключён напрямую к Ubuntu-ПК

После сборки первой недели `sllidar_ros2` уже доступен. Закрой симуляцию.
Разреши своему пользователю доступ к serial:

```bash
sudo usermod -aG dialout "$USER"
```

Полностью выйди из графического сеанса Ubuntu и войди снова, чтобы новая
группа применялась к терминалам. Перезагрузка тоже применяет группы.
В новом локальном терминале:

```bash
cd ~/robot/room_robot_v5_four_weeks
source scripts/env_robot.sh
export ROS_DOMAIN_ID=43
ls -l /dev/serial/by-id/
```

Найди устройство C1 подключением/отключением USB. Затем используй команду
запуска C1 ниже; в другом локальном терминале с тем же окружением и domain 43
проверь `/scan`. Pi и передача файла для этого варианта не нужны.

### Вариант B: C1 подключён к Raspberry Pi

В локальном терминале Ubuntu-ПК передай скачанный архив и подключись:

```bash
room_download_dir=$(xdg-user-dir DOWNLOAD)
scp "$room_download_dir/room_robot_v5_four_weeks.zip" shapturov@192.168.0.20:room_robot_v5_four_weeks.zip
ssh shapturov@192.168.0.20
```

При другом местоположении ZIP измени локальный путь. `192.168.0.20` — известный
адрес Pi; если роутер изменил его, используй новый. Файл попадёт в домашнюю
папку удалённого пользователя.

### Только для варианта B: подготовить драйвер на Pi

Дальше команды выполняются **в SSH Pi**:

```bash
hostname -I
cat /etc/os-release
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y unzip build-essential python3-colcon-common-extensions python3-rosdep
mkdir -p ~/robot
unzip ~/room_robot_v5_four_weeks.zip -d ~/robot
cd ~/robot/room_robot_v5_four_weeks
```

Если проект уже распакован, используй имеющуюся корректную копию и не перезаписывай
свои настройки. Если `/opt/ros/jazzy/setup.bash` отсутствует, сначала установи
Jazzy по общей инструкции; для этой отдельной проверки достаточно ROS base.

Если rosdep на Pi ещё не инициализирован, один раз `sudo rosdep init`. Затем:

```bash
rosdep update
rosdep install --from-paths ros2_ws/src/sllidar_ros2 \
  --ignore-src -r -y --rosdistro jazzy
cd ros2_ws
colcon build --symlink-install --base-paths src/sllidar_ros2 \
  --packages-select sllidar_ros2
cd ..
sudo usermod -aG dialout "$USER"
```

После изменения группы выйди из SSH через `exit` и подключись заново. Определи
порт C1 по подключению/отключению устройства; не подставляй порт Arduino:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot/room_robot_v5_four_weeks
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43
ls -l /dev/serial/by-id/
```

### Общая команда запуска C1 на выбранной машине

Вариант A: выполняй в подготовленном локальном терминале ПК, пропустив
SSH-команды варианта B. Вариант B: выполняй в подготовленном SSH-терминале Pi.
В обоих случаях выбран domain 43. Для пути, найденного именно у C1:

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/serial/by-id/REPLACE_WITH_C1 \
  serial_baudrate:=460800 frame_id:=laser \
  inverted:=false angle_compensate:=true scan_mode:=Standard
```

`REPLACE_WITH_C1` замени настоящим именем. Если USB-адаптер не даёт уникального
by-id, проверь `/dev/serial/by-path/` и используй постоянный физический USB-порт.

Для варианта B во втором SSH-окне Pi подготовь тот же ROS/overlay и domain
`43`, затем (в варианте A эти команды выполняются локально на ПК):

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot/room_robot_v5_four_weeks/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43
ros2 topic hz /scan
```

Останови частотную проверку через Ctrl+C, затем прочитай заголовок одного скана:

```bash
ros2 topic echo /scan --once --field header --qos-reliability best_effort
```

Ожидается frame `laser` и обновляющийся поток сканов, обычно около 10 Гц.
При подключении C1 к ПК можно открыть локальный `rviz2`, выбрать Fixed Frame
`laser`, добавить LaserScan `/scan` с Best Effort. Для просмотра скана с Pi на
ПК отдельно проверь ROS-связность сети; в первой неделе достаточно проверки
топика внутри Pi. Совпадающий domain ID сам по себе не гарантирует сетевую связь.
Останови драйвер Ctrl+C после проверки. SLAM по реальному C1 и калибровка
реального шасси — следующий отдельный этап.

## 11. Если что-то не работает

| Наблюдение | Что проверить и сделать |
|---|---|
| Ubuntu оказалась 26.04/22.04 | Остановиться до подключения Noble-репозитория; нужен согласованный вариант ОС и ROS, не смешивать пакеты |
| `ros2: command not found` | Это должно быть окно Ubuntu, затем source ROS; после сборки — `source scripts/env_robot.sh` |
| `build_ros` отказывает из-за venv/Conda | Новый локальный терминал, собрать `week01` до source окружения; не удалять системный Python |
| Package `room_sim` не найден | Сборка завершилась? source overlay выполнен в этом же терминале? правильная копия проекта? |
| Не открывается даже RViz | Графическая сессия Ubuntu, `glxinfo -B`, логи и существующий GPU-драйвер |
| Gazebo ругается на Wayland/currentGLContext | Отдельная проба `QT_QPA_PLATFORM=xcb gz sim shapes.sdf -v 4`; при успехе тот же префикс можно применить к проектному launch |
| Есть EGL warning, но всё рисуется | Сам warning не доказывает отказ; проверить картинку, `/scan`, сообщения и лог Ogre |
| Есть GUI, но нет `/scan`/изображения | Sensor renderer Ogre2, ros_gz_bridge, правильные topics, отсутствие второго launch; проверить реальные сообщения, а не только список имён |
| Сенсоры идут редко, health false | Частота реальной доставки, загрузка GPU/CPU, прогресс `/clock`, `odom_status`; не отключать watchdog ради движения |
| Нет `/odom_rf2o` | `/scan`, `base_link → laser`, логи RF2O и валидные отражения |
| Есть `/odom_rf2o`, но нет здоровой `/odom` | Причина guard в `/room_base/odom_status`; целиком перезапустить после устранения latched fault |
| Карта не видна | `/map`, durability `Transient Local`, Fixed Frame `map`, TF и время для первого обновления SLAM |
| `goal_ab` не проходит ARM-проверку | Launch именно с `simulation_autonomy:=true`, единственный `/room_sim_drive`, health, активный Nav2; никакого аппаратного bridge в этом domain |
| Робот не продолжил после STOP/паузы | Это ожидаемо: старая поездка не должна оживать; отменить задачу, устранить причину, начать новую сессию |
| Сохранённая карта не совпадает со сканом | Правильный файл YAML+image, прежний мир, fresh RF2O и initial pose AMCL с правильным yaw |
| C1 не открывает serial | Права после нового входа в сеанс/SSH, правильный уникальный USB-порт, 460800, отсутствие второго драйвера на этом порте |

OpenGL/Ogre-лог: `~/.gz/rendering/ogre2.log`. Официальный Gazebo troubleshooting
описывает проблемы OpenGL и Wayland; основной sensor renderer проекта — Ogre2.
Принудительный software rendering или переход на другой engine не считаются
доказанным исправлением данного LiDAR/камерного контура.

## 12. План на семь дней и приёмка

| День | Что сделать | Что сохранить |
|---|---|---|
| 1 | Проверка Ubuntu24.04, ROS Jazzy, окно RViz и `glxinfo -B` | Версии и renderer |
| 2 | Распаковать проект в `~/robot`, scoped rosdep, build week01 | Итог colcon и команды окружения |
| 3 | Первый мир, `/clock`, камера, scan, TF и RF2O | Короткие логи частоты и health |
| 4 | Проверить карту и активные Nav2 actions | Снимок RViz со сканом и картой |
| 5 | Последовательный проезд к A/B в симуляции | Лог `goal_ab`, короткое видео поведения |
| 6 | STOP/отмена, полный перезапуск, карта и AMCL | Карта YAML+image, initial pose и поведение после STOP |
| 7 | Повторить чистый запуск; необязательно проверить C1 на ПК/Pi | Список воспроизводимых команд и оставшихся ошибок |

Критерии завершения недели:

- Симуляция запускается повторно из подготовленных терминалов; два launch не
  конкурируют за одинаковые темы и сервисы.
- Камера и LiDAR действительно выдают сообщения, `/clock` движется, RF2O и guard
  здоровы, TF и карта согласованы. `/sim/odom` не подменяет рабочую одометрию.
- A и B достигаются через Nav2; прерывание останавливает выход, отменяет owned
  goals либо требует полного перезапуска при неизвестном исходе отмены.
- После STOP/отказа движение не восстанавливается без новой явной задачи.
- Карта сохранена с изображением; AMCL на свежем мире проверен с правильной
  начальной позой, а не с позой конца прошлой поездки.
- Оставшиеся ошибки записаны с логами и условиями воспроизведения. Отсутствие
  ROS-ошибок само по себе не заменяет наблюдение маршрута и остановки.

Это подготовленная инструкция по реальным API и исходникам проекта.
**Colcon/ROS2/Gazebo и реальные поездки в среде подготовки не запускались.**
Проверка исходников и offline-тесты не заменяют испытание на твоём ПК.

## 13. Что читать в коде и что пока настраивать

Выполняй чтение в порядке потока данных:

| Файл/каталог | Что объясняет |
|---|---|
| `scripts/install_system.sh` | Системные зависимости режима sim/hardware |
| `scripts/build_ros.sh` | Выбор пакетов по неделе и сборка системным Python |
| `scripts/env_robot.sh` | Подключение ROS, overlay, venv и корня проекта |
| `ros2_ws/src/room_sim/launch/sim.launch.py` | Общий запуск, mapping/localization, симуляционное время, параметры привода |
| `ros2_ws/src/room_sim/worlds/room.sdf` | Сцена комнаты и подключение моделей |
| `ros2_ws/src/room_sim/models/room_robot/model.sdf` | Физика корпуса/колёс, датчики и плагины Gazebo |
| `ros2_ws/src/room_sim/urdf/room_robot.urdf` | Frames корпуса и сенсоров для TF |
| `ros2_ws/src/room_nav/launch/bringup.launch.py` | RF2O, guard, SLAM/AMCL и Nav2 |
| `ros2_ws/src/room_base/room_base/odom_guard.py` и `odom_core.py` | Приём RF2O, контроль времени и качества, рабочие `/odom`/health |
| `ros2_ws/src/room_sim/room_sim/drive_adapter.py` и `safety.py` | ARM, STOP, ограничение команд и watchdog виртуального привода |
| `ros2_ws/src/room_sim/room_sim/goal_ab.py` | Nav2 action client и последовательные цели |
| `ros2_ws/src/room_sim/room_sim/simulation_goal.py` и `action_lifecycle.py` | Проверки ARM, владение action-целями и отмена |

Поток координат: скан `/scan` поступает в RF2O; его `/odom_rf2o` проверяет
odom guard и публикует рабочую `/odom` и `odom → base_link`. SLAM Toolbox
в mapping или AMCL в localization публикует `map → odom`. URDF задаёт ветви
`base_link → laser` и камеры. Nav2 использует карту, эти TF и текущие препятствия.

RF2O оценивает смещение по изменению отражений между сканами. Без энкодеров
это возможно, но наблюдаемость хуже в длинных однообразных коридорах, при
резких движениях или большом числе движущихся объектов. Именно поэтому
качество одометрии проверяется отдельно от факта публикации топика.

Для первого запуска оставь геометрию и параметры исходного симуляционного мира.
Менять нужно только путь к установленному проекту, domain ID при необходимости
и свободные цели. После успешного исходного испытания меняй по одному параметру
и фиксируй результат. Параметры URDF и SDF должны описывать одно и то же
положение сенсоров. Размеры, TF и PWM настоящего шасси настраиваются отдельно
по измерениям; чисел из симуляции для этого недостаточно.

## Первичные источники и связанные инструкции

- [ROS 2 Jazzy: Ubuntu deb installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- [Исходник текущей ROS apt-source инструкции](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).
- [Gazebo: совместимые пары с ROS](https://gazebosim.org/docs/harmonic/ros_installation/),
  [OpenGL/Wayland troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/).
- [Slamtec: официальный sllidar_ros2](https://github.com/Slamtec/sllidar_ros2).
- [Базовая установка проекта](INSTALL_RU.md), [архитектура недели 1](WEEK01_RU.md),
  [привод: отдельный следующий этап](BASE_HARDWARE_RU.md).
  Настройка физической USB-камеры — `docs/CAMERA_RU.md` полного проекта и срезов начиная с недели 2.
