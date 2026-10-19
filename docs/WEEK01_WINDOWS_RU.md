# Неделя 1 с Windows: WSL2 → Gazebo → RF2O → SLAM → Nav2

Эта инструкция начинается с твоей текущей схемы: на ПК Windows, на Raspberry Pi
Ubuntu и SSH `shapturov@192.168.0.20`. Основной результат недели — **виртуальный
робот едет к двум точкам в виртуальной комнате**, получает лазерную одометрию и
строит карту. Всё это сначала работает локально на ПК в Ubuntu 24.04 через WSL2.

Raspberry Pi на этом этапе не управляет симулятором. Настоящий RPLIDAR C1 можно
проверить на Pi отдельно по последнему разделу. Прошивать Arduino, подключать
реальные моторы, устанавливать PyTorch/Ollama и обучать модели для этой недели
не требуется. Симуляция не является готовой калибровкой реального привода.

## 1. Где выполнять команды

| Окно | Как открыть | Что здесь выполняется |
|---|---|---|
| PowerShell Windows | Пуск → PowerShell; для установки WSL — от администратора | `wsl`, передача файла через `scp` |
| Ubuntu WSL на ПК | `wsl -d Ubuntu-24.04` из PowerShell | Установка ROS, сборка проекта, Gazebo, RViz, все основные проверки недели |
| Вторая/третья Ubuntu WSL | Ещё одно окно PowerShell → тот же `wsl -d Ubuntu-24.04` | Проверки топиков, цели, STOP, сохранение карты |
| SSH Raspberry Pi, необязательно | `ssh shapturov@192.168.0.20` | Только отдельная проверка настоящего C1 |

Не вставляй блоки `sudo apt ...` для симуляции в SSH Pi. Эти команды должны
выполняться в **локальной Ubuntu WSL на ПК**. Внутри Linux `hostname` помогает
проверить машину; название дистрибутива — `cat /etc/os-release`.

На ПК используем `ROS_DOMAIN_ID=42`, на необязательном стенде C1 на Pi — `43`.
Это два отдельных ROS-графа: они не должны одновременно подставлять `/scan`
друг другу. Сетевое объединение Pi и WSL оставляем следующему этапу.

## 2. Установить именно Ubuntu 24.04 в WSL2

Нужны Windows 11 либо Windows 10 build 19044+ для WSLg, аппаратная виртуализация
и современный Windows-драйвер твоей видеокарты. WSLg позволяет Linux-приложениям
открывать окна на рабочем столе Windows и использовать аппаратный OpenGL.
Это не означает, что произвольная связка драйвера и Ogre2 уже проверена.

В PowerShell **от администратора**:

```powershell
wsl --list --online
wsl --install -d Ubuntu-24.04
```

Выбираем название `Ubuntu-24.04` явно. Обычное `wsl --install` может установить
другую актуальную Ubuntu; проект рассчитан на **24.04 / Noble / ROS 2 Jazzy**.
Если установщик просит перезагрузку Windows, перезагрузись. При первом запуске
Ubuntu создай Linux-пользователя и пароль. Это отдельная учётная запись;
пароль не обязан совпадать с Windows или Pi. При вводе Linux-пароля символы
и звёздочки в терминале не отображаются — это нормально.

Затем в PowerShell:

```powershell
wsl --update
wsl --shutdown
wsl --list --verbose
wsl -d Ubuntu-24.04
```

`wsl --shutdown` останавливает все работающие WSL-дистрибутивы; здесь он нужен
до запуска симуляции, чтобы применилось обновление. В колонке VERSION для
`Ubuntu-24.04` должно быть `2`. Если там `1`, закрой Ubuntu и выполни в PowerShell:

```powershell
wsl --set-version Ubuntu-24.04 2
wsl -d Ubuntu-24.04
```

Если возникает ошибка о виртуализации, проверь «Виртуализация» в Диспетчере задач
Windows → Производительность → ЦП и требования из официальной инструкции WSL.
Не переходи к ROS, пока Ubuntu не запускается.

В открывшейся **локальной Ubuntu WSL**:

```bash
cd ~
cat /etc/os-release
python3 --version
```

Ожидается `VERSION_ID="24.04"`, `VERSION_CODENAME=noble`, Python `3.12`.
Официальные инструкции: [установка WSL](https://learn.microsoft.com/en-us/windows/wsl/install),
[WSLg и драйвер GPU](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

## 3. Установить ROS 2 Jazzy и проверить графику

Все команды этого раздела — **в Ubuntu WSL**, обычным пользователем, с `sudo`
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
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools mesa-utils unzip
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Проверка OpenGL:

```bash
glxinfo -B
rviz2
```

Должно открыться окно RViz на рабочем столе Windows. Закрой его перед следующим
этапом. В `glxinfo -B` посмотри название renderer и OpenGL: `llvmpipe` означает
программный рендеринг, который может быть слишком медленным для этой сцены.
WSLg может показывать renderer через D3D12 — это не ошибка само по себе.
Проверка `nvidia-smi`, если доступна, не заменяет проверку OpenGL.

Если окна Linux не открываются, сначала исправляй WSLg/Windows-драйвер GPU.
Не подставляй случайный `DISPLAY` из инструкций для старых внешних X-серверов:
WSLg настраивает display самостоятельно.

ROS Jazzy ставится на Ubuntu 24.04 по [официальной инструкции ROS](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
При конфликтах установки `ros-dev-tools` проверь наличие `noble-updates` и
`noble-backports` в `/etc/apt/sources.list.d/ubuntu.sources`, как описано там.

## 4. Перенести архив в Linux и поставить зависимости проекта

Скачай `room_robot_v5_four_weeks.zip` на Windows. В **Ubuntu WSL**:

```bash
mkdir -p ~/robot
cd ~/robot
explorer.exe .
```

Откроется именно Linux-папка `~/robot` в Проводнике Windows. Скопируй в неё
скачанный ZIP через Проводник. Затем вернись в терминал Ubuntu:

```bash
ls -lh room_robot_v5_four_weeks.zip
unzip room_robot_v5_four_weeks.zip
cd ~/robot/room_robot_v5_four_weeks
bash scripts/install_system.sh sim
```

Если имя скачанного архива отличается, подставь его в `ls` и `unzip`.
В инструкции дальше предполагается папка `~/robot/room_robot_v5_four_weeks`.
При повторной установке не перезаписывай свои карты и конфигурации вслепую.

Храни распакованный проект в Linux `~/robot`, а не собирай непосредственно
в `/mnt/c/Users/...`: для Linux-сборок Microsoft рекомендует Linux filesystem.
[Объяснение и открытие через Проводник](https://learn.microsoft.com/en-us/windows/wsl/filesystems).

`install_system.sh sim` ставит Nav2, SLAM Toolbox, инструменты сборки и
`ros-jazzy-ros-gz`. Это официальная связка **Jazzy + Gazebo Harmonic**.
Gazebo Classic, отдельная Windows-версия Gazebo и отдельный OSRF-репозиторий
для этого пути не нужны. [Совместимость ROS/Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).

До сборки проекта можно отдельно проверить Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
gz sim shapes.sdf -v 4
```

Откроется стандартный мир с простыми фигурами. Закрой его полностью перед
проектным launch. Это проверка GUI/базового мира, **не** проверка камеры,
GPU LiDAR, RF2O и Nav2. В проектной сцене эти компоненты проверяются отдельно.

## 5. Собрать только пакеты первой недели

Открой **новый терминал Ubuntu WSL без venv/Conda**. Сначала ROS-сборка,
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

Открой ещё одно окно PowerShell и войди в **тот же** WSL-дистрибутив:

```powershell
wsl -d Ubuntu-24.04
```

Затем в Ubuntu:

```bash
cd ~/robot/room_robot_v5_four_weeks
source scripts/env_robot.sh
export ROS_DOMAIN_ID=42
```

Это обязательный префикс для всех следующих терминалов ПК. Скрипт подключает
ROS и colcon overlay, активирует venv и добавляет исходники в `PYTHONPATH`.
Он должен выполняться через **source**, не `bash`.

Необязательный отдельный тест ROS: в двух подготовленных WSL-терминалах
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

## 10. Необязательно: проверить настоящий C1 на Pi отдельно

Этот раздел **не заменяет симуляцию** и не запускает реальные моторы. Здесь
нужны только Pi, его Ubuntu/ROS Jazzy, C1 и USB-подключение. Не запускай на Pi
`room_nav bringup` с моторным мостом ради проверки скана.

### Передать исходники

Выбери один способ. В **Windows PowerShell**:

```powershell
scp "$env:USERPROFILE\Downloads\room_robot_v5_four_weeks.zip" shapturov@192.168.0.20:room_robot_v5_four_weeks.zip
ssh shapturov@192.168.0.20
```

Если архив уже лежит в Linux `~/robot`, можно вместо этого в **Ubuntu WSL**:

```bash
scp ~/robot/room_robot_v5_four_weeks.zip shapturov@192.168.0.20:room_robot_v5_four_weeks.zip
ssh shapturov@192.168.0.20
```

Оба варианта копируют файл в домашний каталог удалённого пользователя.
При другом имени или местоположении архива измени локальный путь.
`192.168.0.20` — текущий IP Pi, не самой WSL; если роутер изменит его, замени
адрес в SSH/scp-командах.

### Подготовить только драйвер C1

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

Для пути, найденного именно у C1:

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/serial/by-id/REPLACE_WITH_C1 \
  serial_baudrate:=460800 frame_id:=laser \
  inverted:=false angle_compensate:=true scan_mode:=Standard
```

`REPLACE_WITH_C1` замени настоящим именем. Если USB-адаптер не даёт уникального
by-id, проверь `/dev/serial/by-path/` и используй постоянный физический USB-порт.

Во втором SSH-окне Pi подготовь тот же ROS/overlay и domain `43`, затем:

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
Для просмотра физического скана в локальном RViz на ПК потребуется отдельно
настроенная ROS-сеть Pi ↔ WSL. Для этой недели достаточно проверки внутри Pi;
одного совпадающего domain ID для прохождения DDS через WSL NAT недостаточно.
Останови драйвер Ctrl+C после проверки. SLAM по реальному C1 и калибровка
реального шасси — следующий отдельный этап.

## 11. Если что-то не работает

| Наблюдение | Что проверить и сделать |
|---|---|
| Ubuntu оказалась 26.04/22.04 | Вернуться к явному `Ubuntu-24.04`; не смешивать пакеты разных Ubuntu/ROS |
| `ros2: command not found` | Это должно быть окно Ubuntu, затем source ROS; после сборки — `source scripts/env_robot.sh` |
| `build_ros` отказывает из-за venv/Conda | Новый WSL-терминал, собрать `week01` до source окружения; не удалять системный Python |
| Package `room_sim` не найден | Сборка завершилась? source overlay выполнен в этом же терминале? правильная копия проекта? |
| Не открывается даже RViz | WSL VERSION2, обновление WSL/Windows-драйвера, WSLg и `glxinfo -B` |
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
| C1 на Pi не открывает serial | Права после нового SSH-входа, правильный уникальный USB-порт, 460800, отсутствие второго драйвера на этом порте |

OpenGL/Ogre-лог: `~/.gz/rendering/ogre2.log`. Официальный Gazebo troubleshooting
описывает проблемы OpenGL и Wayland; основной sensor renderer проекта — Ogre2.
Принудительный software rendering или переход на другой engine не считаются
доказанным исправлением данного LiDAR/камерного контура.

## 12. План на семь дней и приёмка

| День | Что сделать | Что сохранить |
|---|---|---|
| 1 | WSL2 + Ubuntu24.04, ROS Jazzy, окно RViz и `glxinfo -B` | Версии и renderer |
| 2 | Распаковать в Linux filesystem, scoped rosdep, build week01 | Итог colcon и команды окружения |
| 3 | Первый мир, `/clock`, камера, scan, TF и RF2O | Короткие логи частоты и health |
| 4 | Проверить карту и активные Nav2 actions | Снимок RViz со сканом и картой |
| 5 | Последовательный проезд к A/B в симуляции | Лог `goal_ab`, короткое видео поведения |
| 6 | STOP/отмена, полный перезапуск, карта и AMCL | Карта YAML+image, initial pose и поведение после STOP |
| 7 | Повторить чистый запуск; необязательно проверить C1 на Pi | Список воспроизводимых команд и оставшихся ошибок |

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
**Colcon/ROS2/Gazebo и реальные поездки в среде подготовки не запускались**;
поддержка окон WSLg не равна проверенной работе сенсоров и всей автономности.

## Первичные источники и связанные инструкции

- [Microsoft: установка WSL и выбор дистрибутива](https://learn.microsoft.com/en-us/windows/wsl/install).
- [Microsoft: WSLg, требования и Windows-драйверы GPU](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).
- [Microsoft: Linux filesystem и доступ через Проводник](https://learn.microsoft.com/en-us/windows/wsl/filesystems).
- [ROS 2 Jazzy: Ubuntu deb installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- [Исходник текущей инструкции ROS apt-source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).
- [Gazebo: совместимые пары с ROS](https://gazebosim.org/docs/harmonic/ros_installation/),
  [стандартный shapes.sdf](https://gazebosim.org/docs/harmonic/getstarted/),
  [OpenGL/Wayland troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/).
- [Slamtec: официальный sllidar_ros2](https://github.com/Slamtec/sllidar_ros2).
- [Базовая установка проекта](INSTALL_RU.md), [код и архитектура недели 1](WEEK01_RU.md),
  [привод: отдельный следующий этап](BASE_HARDWARE_RU.md).
  Настройка USB-камеры для настоящего робота описана в `docs/CAMERA_RU.md` полного проекта и срезов начиная с недели 2.
