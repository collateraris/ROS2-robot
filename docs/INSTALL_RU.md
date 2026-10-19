# Установка окружения v5

Целевая система: Ubuntu 24.04, ROS 2 Jazzy, системный Python 3.12, Gazebo Harmonic для симуляции. Humble и Jazzy не смешивать; для этой поставки команды Humble не предусмотрены. Начать всё на одной Ubuntu-системе. Разнесение сенсоров/нейросетей по Wi-Fi — следующий шаг после работающего локального запуска.

## 1. ROS2 Jazzy

Если `source /opt/ros/jazzy/setup.bash; ros2 --help` уже работает, переустановка не требуется.
Для чистой Ubuntu 24.04:

```bash
sudo apt update
sudo apt install -y curl software-properties-common locales
sudo add-apt-repository universe
locale
```

Вывод locale должен использовать UTF-8. При отсутствии:

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Официальный apt-source пакет:

```bash
room_ros_repo_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${room_ros_repo_version}/ros2-apt-source_${room_ros_repo_version}.noble_all.deb" -o /tmp/room-ros2-apt-source.deb
sudo dpkg -i /tmp/room-ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Официальные инструкции: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## 2. Распаковка и apt-зависимости проекта

```bash
mkdir -p ~/robot
unzip ~/Downloads/room_robot_v5_four_weeks.zip -d ~/robot
cd ~/robot/room_robot_v5_four_weeks
bash scripts/install_system.sh sim
```

`sim` устанавливает Gazebo/ros_gz, robot_state_publisher и xacro вместе с Nav2/SLAM. Для одной аппаратной системы без симулятора можно выбрать `hardware`.

## 3. Собрать ROS-пакеты до venv

Использовать новый терминал с системным Python, без активного Conda/venv.

```bash
cd ~/robot/room_robot_v5_four_weeks
source /opt/ros/jazzy/setup.bash
bash scripts/fetch_ros_drivers.sh
```

Если rosdep ещё не был настроен, один раз `sudo rosdep init`. Затем:

```bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro jazzy
bash scripts/build_ros.sh all
```

Для аппаратной установки без ros_gz: `bash scripts/build_ros.sh hardware`; rosdep при этом запускать только для `room_base room_nav room_perception room_tasks rf2o_laser_odometry sllidar_ros2`, например явно перечислив эти каталоги после --from-paths. Иначе rosdep обоснованно поставит зависимости симулятора из room_sim/room_eval.

Исправленный build_ros временно выключает Bash nounset только при source ROS. Сборка проверяет новые ROS-пакеты через colcon. Реальная сборка в среде автора не выполнена, потому что ROS 2 runtime отсутствует; при первом запуске сохранить полный вывод colcon.

## 4. Окружение Python

Для недели1 достаточно:

```bash
bash scripts/setup_python.sh base
source scripts/env_robot.sh
```

Для воспроизводимой CPU-проверки всех Python-модулей:

```bash
bash scripts/setup_python.sh all cpu
source scripts/env_robot.sh
python -c 'import torch,transformers; print(torch.__version__,transformers.__version__,torch.cuda.is_available())'
```

CPU подходит для проверки кода, но своевременное восприятие/планирование в движении не гарантируется. Для GPU сначала создать `.venv` командой `/usr/bin/python3 -m venv --system-site-packages .venv`, затем установить совместимый с драйвером PyTorch по https://pytorch.org/get-started/locally/ через `.venv/bin/python -m pip ...`. После этого:

```bash
bash scripts/setup_python.sh all existing
source scripts/env_robot.sh
python -c 'import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name())'
```

В extras зафиксирован Transformers 4.52.3, под его API написан GroundingDINO-адаптер. `torch` >=2.6,<3. Установка новой major-версии Transformers требует проверки API, не меняй её автоматически. Симулятор, детектор и VLM могут конкурировать за одну GPU; измерять задержку, не обещать суммарное потребление VRAM по размеру скачанного файла.

## 5. Каждый новый терминал

```bash
cd ~/robot/room_robot_v5_four_weeks
source scripts/env_robot.sh
```

Скрипт source ROS+colcon overlay, активирует venv, добавляет корень проекта в PYTHONPATH и задаёт ROS_DOMAIN_ID=42 только если он не задан. Он должен быть именно source, не bash: переменные нужны текущему терминалу.

Проверки:

```bash
python -c 'import rclpy,room_base.core; print("ROS Python OK")'
ros2 pkg prefix room_sim
ros2 pkg prefix room_perception
ros2 pkg prefix room_tasks
ros2 pkg prefix room_eval
```

ROS console scripts имеют системный Python в shebang. Поэтому ML launch передаёт `python_executable:="$ROOM_PROJECT_ROOT/.venv/bin/python"`. Одной активации venv для такого console script недостаточно. Root Python-пакеты видны через PYTHONPATH; ROS-библиотеки — через --system-site-packages и source overlay.

## 6. Локальная VLM через Ollama (неделя3)

Установить Ollama по https://ollama.com/download/linux . Перед установкой можно скачать и прочитать официальный install.sh, затем выполнить его локально; скрипт установки не вызывается проектом автоматически. Проверить службу:

```bash
ollama --version
ollama pull qwen2.5vl:7b
curl -fsS http://127.0.0.1:11434/api/tags
```

Для текстового планировщика без изображения допустим `qwen2.5:7b-instruct`; выбор модели явно задаётся в launch и отчёте. Это локальный HTTP API. Проект не требует OpenAI API key и не отправляет кадры наружу по умолчанию. Не выставлять Ollama в публичную сеть для локального стенда.

## 7. C1/Arduino на реальном роботе

```bash
sudo usermod -aG dialout "$USER"
```

Полностью выйти/войти в пользовательский сеанс. Определять C1 и Arduino поочерёдным подключением через `/dev/serial/by-id/` или, при неуникальных id, `/dev/serial/by-path/`. C1: 460800, Arduino:115200. Схема без энкодеров из v4 сохранена. Аппаратные пины, прошивка V5, калибровка PWM и STOP: [BASE_HARDWARE_RU.md](BASE_HARDWARE_RU.md). Первую карту строить с `start_bridge:=false` и отключённым питанием моторов.

## 8. Проверка окружения и кода

```bash
python scripts/doctor.py --ros --sim
bash scripts/verify_offline.sh
```

Для ML дополнительно `python scripts/doctor.py --perception --ollama`. Doctor сообщает наличие зависимостей, не выполняет движение. Offline-тесты не заменяют запуск ROS/Gazebo. Если `/clock` не продвигается, снять паузу Gazebo; sim-time узлы не должны получать wall-time sensor stamps.

Неделя 1: [WEEK01_RU.md](WEEK01_RU.md). Далее WEEK02/03/04 в том же каталоге.
