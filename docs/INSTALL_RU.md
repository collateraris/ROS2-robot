# Установка: Raspberry Pi, Windows 11 и телефон

Сам робот работает на **Ubuntu 24.04 LTS arm64 + ROS 2 Jazzy**. Телефон открывает страницу на Pi: получает JPEG-видео и отправляет команды по WebSocket. Arduino подключён к Pi по USB и управляет моторами. Компьютер нужен для установки, прошивки и разработки; после запуска в `tmux` по разделу 6 его можно выключить. Pi и Wi-Fi-роутер должны оставаться включёнными; если сеть раздаёт сам компьютер, выключать его нельзя. RTX 4090 для этого ручного управления не требуется.

В примерах `192.168.1.42` — пример адреса Pi, `robot` — пример имени пользователя. **Замените их своими значениями.** Инструкция не создаёт пользователя `robot`: если вы вошли на Pi как `ubuntu`, используйте `ubuntu@АДРЕС_PI`. Все команды Bash выполняются на указанной машине; команды PowerShell — в Windows. До первой проверки поднимите колёса и прочитайте [подключение оборудования](HARDWARE_RU.md).

## 1. Ubuntu на Raspberry Pi

Нужны Raspberry Pi 4/5, карта microSD или другой поддерживаемый загрузочный накопитель, подходящее питание, USB-камера, Arduino и Wi-Fi-роутер. Для первого запуска удобны HDMI-монитор, клавиатура и временное соединение Ethernet.

**На Windows:** установите [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Выберите свою Pi и образ **Ubuntu Server 24.04 LTS, 64-bit / arm64**. Если его нет в списке, скачайте официальный образ Ubuntu 24.04 для Raspberry Pi и используйте выбор собственного образа. Не подменяйте его обычным ISO для PC, 32-битной системой или Ubuntu 26.04: команды ниже рассчитаны на Jazzy/Noble. Запись образа удалит данные с выбранного накопителя, поэтому проверьте его имя и размер.

Если Imager предлагает настройки пользователя, Wi-Fi и SSH для выбранного Ubuntu-образа, заполните их. Поддержка настройки образов различается; гарантированный путь первого запуска — монитор, клавиатура и Ethernet. После загрузки дождитесь завершения cloud-init. На немодифицированном серверном образе Canonical используются `ubuntu` / `ubuntu` с обязательной сменой пароля; если пользователя меняли в настройках образа, используйте заданные данные.

**В терминале Pi:**

```bash
cat /etc/os-release
uname -m
sudo apt update
sudo apt upgrade -y
sudo apt install -y openssh-server unzip
sudo systemctl enable --now ssh
hostname -I
```

Ожидаются Ubuntu 24.04 и `aarch64`. При запросе перезагрузки выполните `sudo reboot`, затем войдите снова. Сохраните IPv4-адрес Pi. Для постоянного адреса удобно настроить DHCP reservation на домашнем роутере.

### Если Wi-Fi ещё не настроен

Выполняйте этот раздел с локальной клавиатуры Pi. Сначала найдите имя беспроводного интерфейса:

```bash
ip -br link
sudo ls /etc/netplan
```

Если Wi-Fi уже описан в существующем Netplan-файле, отредактируйте именно его, не создавая второе противоречащее описание. Для свежей Ubuntu Server с `networkd`, когда Wi-Fi ещё нигде не задан, создайте файл:

```bash
sudo nano /etc/netplan/90-robot-wifi.yaml
```

Содержимое — замените `wlan0`, SSID и пароль; отступы только пробелами:

```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YOUR_WIFI_SSID":
          password: "YOUR_WIFI_PASSWORD"
```

```bash
sudo chmod 600 /etc/netplan/90-robot-wifi.yaml
sudo netplan try
ip -4 -br address
```

Подтвердите рабочую конфигурацию в диалоге `netplan try`. Этот пример предназначен для Ubuntu Server с `networkd`; в Ubuntu Desktop проще выбрать сеть через графические настройки NetworkManager. Телефон и Pi должны быть в сети, в которой разрешено соединение клиентов между собой: гостевая Wi-Fi-сеть часто его блокирует.

## 2. SSH и перенос проекта с Windows

**PowerShell Windows:**

```powershell
ssh robot@192.168.1.42
```

Подтвердите ключ именно своего Pi при первом подключении. После входа этот терминал уже выполняет команды **на Pi**. Командой `exit` можно вернуться в Windows. В Windows 11 клиент OpenSSH обычно доступен; если команды `ssh`/`scp` нет, установите компонент OpenSSH Client в дополнительных компонентах Windows.

Распакуйте **полный архив проекта** на Windows. Проводник может добавить внешнюю папку с именем архива: нужна внутренняя папка `robot_wifi_teleop`, непосредственно содержащая `arduino`, `docs`, `ros2_ws` и `scripts`. Для команд ниже разместите именно её в Downloads; не должна получиться вложенность `robot_wifi_teleop/robot_wifi_teleop`. В отдельном **PowerShell Windows** сначала проверьте путь:

```powershell
Test-Path "$env:USERPROFILE\Downloads\robot_wifi_teleop\scripts\install_jazzy.sh"
```

Ожидается `True`. Если получилось `False`, найдите правильную внутреннюю папку; не переносите пустую внешнюю оболочку. Затем скопируйте проект целиком:

```powershell
scp -r "$env:USERPROFILE\Downloads\robot_wifi_teleop" robot@192.168.1.42:~/
```

Дальше ожидается корень проекта `~/robot_wifi_teleop` на Pi. Проверьте в **SSH-сеансе Pi**:

```bash
ls ~/robot_wifi_teleop/scripts/install_jazzy.sh
```

Если файла нет, исправьте расположение папки перед следующими шагами. Сначала прошейте Arduino по HARDWARE_RU.md, затем соедините его USB с Pi.

## 3. ROS 2 Jazzy и зависимости на Pi

В комплекте есть установщик для свежей Ubuntu 24.04. **Терминал Pi / SSH:**

```bash
cd ~/robot_wifi_teleop
bash scripts/install_jazzy.sh pi
```

Изучить или выполнить установку вручную можно по следующему разделу. Это **альтернатива** установщику, повторять оба варианта подряд не нужно. Не устанавливайте `rclpy` через pip: здесь используются согласованные системные пакеты Ubuntu/ROS.

### Ручная установка вместо скрипта

**Pi / SSH, Bash:**

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl python3
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-cv-bridge ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs python3-opencv python3-aiohttp python3-serial python3-yaml v4l-utils tmux
```

Если получение версии `ROS_APT_SOURCE_VERSION` или скачивание завершилось ошибкой, остановитесь и проверьте подключение; не продолжайте установку с пустой версией. Пакет `ros2-apt-source` добавляет репозиторий и ключи ROS. Если ROS уже установлен старым способом и apt сообщает о конфликтующих источниках или `Signed-By`, исправьте дубли в `/etc/apt/sources.list.d/` по официальной инструкции ROS, а не добавляйте очередную копию источника.

Разрешите обычному пользователю доступ к USB Arduino и камере:

```bash
sudo usermod -aG dialout,video "$USER"
```

**Выйдите из сеанса и войдите снова**; уже запущенная оболочка не получает новые группы автоматически. Затем:

```bash
groups
source /opt/ros/jazzy/setup.bash
ros2 --help
```

В списке групп должны быть `dialout` и `video`. Запуск ROS от root не нужен.

## 4. Найти Arduino и устройство камеры

**Pi / SSH:**

```bash
ls -l /dev/serial/by-id/
v4l2-ctl --list-devices
```

Предпочтительно использовать найденный полный путь Arduino `/dev/serial/by-id/...`: он устойчивее, чем порядковый `/dev/ttyACM0`. Если USB-адаптер не создаёт `by-id`, найдите реальный `/dev/ttyACM*` или `/dev/ttyUSB*` и используйте его. Проект не угадывает порт автоматически. Arduino Serial Monitor и другие владельцы порта должны быть закрыты.

Для выбранной камеры, например `/dev/video0`:

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=10 --stream-to=/dev/null
```

Используйте путь, который действительно выдаёт кадры. Некоторые камеры создают несколько устройств, включая узлы метаданных. При наличии подходящего `/dev/v4l/by-id/...-video-index0` его тоже можно передать в launch. Поддержка конкретного режима MJPG, 640×480 и 15 fps зависит от камеры; список форматов показывает доступные варианты.

Если камера на CSI-шлейфе, остановитесь на выборе драйвера: этот комплект захватывает USB UVC через V4L2. Для неизвестной CSI-камеры совместимость Ubuntu 24.04 с данным узлом не заявлена; подробнее в HARDWARE_RU.md.

## 5. Собрать ROS-пакет

**Pi / SSH:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_wifi_teleop/ros2_ws
```

Если `rosdep` ещё ни разу не инициализировался на этой системе, выполните один раз:

```bash
sudo rosdep init
```

Если он сообщает, что файл `20-default.list` уже существует, повторная инициализация не нужна. Продолжите:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Не копируйте каталоги `build`, `install` и `log` между Windows, WSL и Pi: пакет собирается отдельно на каждой машине. При добавлении или изменении конфигурационных файлов после сборки повторите `colcon build --symlink-install`.

## 6. Запуск и управление с телефона

Первый запуск — с поднятыми колёсами. Используйте `tmux`, чтобы процесс на Pi продолжал работать после закрытия SSH. Он устанавливается скриптом и ручной командой выше. В новом **терминале Pi / SSH** создайте сеанс:

```bash
tmux new -s robot
```

Откроется оболочка внутри `tmux`; следующие команды запуска выполняйте **в ней**. Если сеанс `robot` уже существует, вместо создания используйте `tmux attach -t robot` и убедитесь, что робот ещё не запущен: второй экземпляр launch запускать нельзя.

Короткий запуск внутри `tmux`:

```bash
cd ~/robot_wifi_teleop
bash scripts/run_robot.sh serial_port:=/dev/serial/by-id/REAL_ID camera_device:=/dev/video0 namespace:=robot
```

Скрипт запросит пароль, подключит ROS-среду и выставит локальное обнаружение с доменом 42. Эквивалентный ручной запуск **в том же сеансе `tmux`, вместо скрипта**, а не дополнительно к нему:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_wifi_teleop/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
read -rsp 'Пароль web, минимум 8 символов: ' ROBOT_PASSWORD
printf '\n'
export ROBOT_PASSWORD
ros2 launch robot_web_teleop robot.launch.py serial_port:=/dev/serial/by-id/REAL_ID camera_device:=/dev/video0 namespace:=robot
```

`REAL_ID` замените настоящим именем из раздела 4. При необходимости измените путь камеры. Пароль вводится без отображения и не попадает как аргумент в историю команд. Запомните его для входа на телефоне. Узлам нужен обычный системный час Pi; не включайте `use_sim_time` для настоящего робота.

После появления сообщений запуска нажмите **`Ctrl+B`, отпустите обе клавиши, затем нажмите `D`**. Это отсоединит терминал от `tmux`, сохранив работу узлов на Pi. Теперь можно закрыть SSH и выключить компьютер. Не нажимайте `Ctrl+C` для отсоединения — эта комбинация останавливает launch.

Чтобы вернуться к журналу запуска, снова подключитесь из Windows командой `ssh robot@192.168.1.42`, затем **на Pi** выполните:

```bash
tmux attach -t robot
```

На телефоне подключитесь к той же домашней Wi-Fi-сети и откройте **`http://192.168.1.42:8080`**, подставив адрес Pi. Введите пароль. Проверьте живую картинку и готовность привода, нажмите **«Взять управление»**, затем держите и перемещайте джойстик. Отпускание джойстика обнуляет движение; **«СТОП»**, потеря соединения или устаревшее видео отключают управление. После сбоя отпустите джойстик, дождитесь свежей картинки и снова нажмите «Взять управление».

Видео передаётся последовательными запросами JPEG примерно до 10 раз в секунду. Страница подтверждает кадр только после декодирования. Если телефон не подтверждает свежий кадр около 0.7 секунды, движение прекращается, даже когда камера на Pi продолжает работать. Закрытый/заблокированный телефон или фоновая вкладка могут перестать подтверждать кадры; это нормальная причина остановки.

Сервер слушает порт 8080. Этот комплект рассчитан на доверенную локальную сеть: **HTTP не шифрует пароль и видео**, поэтому не пробрасывайте порт 8080 из интернета. Для первого запуска не нужны rosbridge, web_video_server, отдельное приложение телефона или ROS на телефоне. Не запускайте второй процесс, публикующий команды в тот же `cmd_vel`.

Для остановки нажмите «СТОП» на телефоне, отпустите джойстик, присоединитесь к сеансу через `tmux attach -t robot` и нажмите **`Ctrl+C` внутри него**. Дождитесь завершения узлов. Команда `exit` после этого закроет оставшуюся оболочку и сеанс `tmux`. Перед работой с проводами отключите моторное питание.

`tmux` сохраняет процесс при разрыве SSH, но **не после перезагрузки или отключения питания Pi**. После каждого такого включения снова подключитесь по SSH, создайте `tmux new -s robot` и запустите скрипт. Автоматического запуска при загрузке в комплекте нет. Если запускать launch напрямую в обычной SSH-оболочке без `tmux`, этот SSH-сеанс нужно оставлять открытым.

### Параметры

Файл параметров пакета находится в `ros2_ws/src/robot_web_teleop/config/robot.yaml`. Это обычный YAML с разделами `camera`, `motor`, `web`, который читает launch, а не стандартный ROS-файл для `--params-file`. Значения камеры по умолчанию: `width: 640`, `height: 480`, **`fps: 15.0`**, `jpeg_quality: 70`. ROS различает типы integer/double, поэтому сохраняйте `.0` у параметров, объявленных как float.

Геометрию `wheel_track_m` задайте по своему шасси. `max_wheel_speed_mps` — масштаб преобразования команды в PWM, требующий экспериментальной настройки, а `pwm_limit` — предел мощности 1…140. Без энкодеров фактическая скорость зависит от батареи, нагрузки и поверхности. Уменьшите предел для первой проверки; программное ограничение не исправляет неподходящий силовой драйвер.

Начальное значение ползунка в браузере — **35%**. Оно дополнительно уменьшает команду: при полностью сдвинутом вперёд джойстике и стандартных настройках получается около **31 из 255 PWM**, а при `pwm_limit: 40` — около **9 из 255**. Этого может не хватить для старта моторов даже при исправной связи. Если привод готов, но моторы стоят или гудят, отпустите джойстик. На поднятых колёсах кратковременно увеличивайте ползунок и, если нужно, `pwm_limit` небольшими шагами, проверяя питание и проводку; не держите остановленный мотор под током. После изменения YAML пересоберите пакет и перезапустите launch. Не трактуйте отсутствие движения при малом PWM как доказательство неисправности ROS.

## 7. Проверка и диагностика без DDS через Wi-Fi

Откройте **второй SSH-сеанс на Pi** и задайте ту же ROS-среду, что у запуска:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_wifi_teleop/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 node list
ros2 topic list
ros2 topic info /robot/camera/image_raw/compressed --verbose
ros2 topic echo /robot/drive/ready --once
```

Основные темы:

| Тема | Тип | Назначение |
|---|---|---|
| `/robot/camera/image_raw` | `sensor_msgs/msg/Image` | BGR8, фактические размеры камеры |
| `/robot/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG для веб-сервера |
| `/robot/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Свежая разрешённая команда |
| `/robot/drive/ready` | `std_msgs/msg/Bool` | USB и Arduino подтвердили готовность |

Камера использует `BEST_EFFORT`, очередь глубиной 1. Собственному подписчику изображения задавайте sensor-data QoS; подписчик, требующий `RELIABLE`, не соединится с таким издателем. Узел не повторяет старый кадр как новый. Метка времени — момент завершения чтения на Pi, а не аппаратное время экспозиции камеры.

| Симптом | Что проверить |
|---|---|
| Телефон не открывает страницу | Правильный IP/8080, сервер запущен, телефон и Pi в доступной друг другу сети, нет client isolation |
| Страница открывается, привод не готов | USB-кабель с данными, настоящий serial_port, прошивка, группы, отсутствие Serial Monitor |
| Привод готов, движения нет | Моторное питание, направление/ENA/ENB/STBY, разрешение управления, свежая картинка; слишком малое произведение ползунка и PWM-предела |
| Картинка отсутствует | Нужный V4L2-узел, группа video, поддерживаемый формат, другая программа не заняла камеру |
| Всё сбрасывается при старте моторов | Питание Pi, аккумулятор, силовая проводка и ток драйвера |
| После изменения ROS-среды не видно узлов | Те же namespace/domain/discovery-параметры; при необходимости `ros2 daemon stop`, затем повторить команду |

Если на Pi уже включён UFW или другой firewall, разрешите TCP 8080 и SSH только из своей локальной подсети. Не отключайте весь firewall ради проверки. `LOCALHOST` ограничивает автоматическое ROS-обнаружение и не является защитой от других программ на самом Pi.

USB-захват выполняется в отдельном потоке, поэтому зависший `VideoCapture.read()` не держит ROS-цикл управления. При обычной ошибке узел пытается переоткрыть камеру. V4L2 не гарантирует жёсткий таймаут чтения через OpenCV: при зависании драйвера может понадобиться перезапуск узла/процесса. В это время веб-сервер перестаёт разрешать движение по устаревшему видео.

## 8. ROS 2 на Windows 11 через WSL2 — необязательно для езды

Для настройки робота достаточно PowerShell, SSH и браузера. WSL нужен, если вы хотите учиться ROS, запускать локальные примеры и RViz. Все следующие Linux-команды выполняются в **Ubuntu WSL**, а не на Pi.

**PowerShell от администратора:**

```powershell
wsl --list --online
wsl --install -d Ubuntu-24.04
wsl --update
wsl --list --verbose
```

Перезагрузите Windows, если установщик попросит. Откройте Ubuntu 24.04, создайте Linux-пользователя и пароль. В `wsl --list --verbose` у Ubuntu должна быть версия 2. Если установлена другая Ubuntu, не считайте её автоматически совместимой с Jazzy: нужна именно 24.04. Команда `wsl -d Ubuntu-24.04` открывает нужную систему.

Перенесите папку исходников из Windows Downloads в домашний каталог WSL; имя Windows-пользователя замените своим:

```bash
cp -r "/mnt/c/Users/YOUR_WINDOWS_USER/Downloads/robot_wifi_teleop" ~/
cd ~/robot_wifi_teleop
bash scripts/install_jazzy.sh pc
source /opt/ros/jazzy/setup.bash
```

Для ручной установки используйте общую часть настройки репозитория из раздела 3, затем установите `ros-jazzy-desktop` и `ros-dev-tools`. Для сборки этого проекта в WSL понадобятся также Python/ROS-зависимости из раздела 3, но подключать Arduino и запускать аппаратный launch на PC не требуется.

Проверка в **первом терминале Ubuntu WSL**:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run demo_nodes_cpp talker
```

Во **втором терминале той же Ubuntu WSL**:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run demo_nodes_py listener
```

Должны появляться полученные сообщения. Для проверки GUI остановите один из примеров и запустите `rviz2`. Windows 11 с обновлённым WSL поддерживает Linux GUI через WSLg; отдельный X-сервер и ручное переопределение DISPLAY в базовом варианте не нужны.

**Эти WSL-узлы локальные и не обязаны видеть Pi.** У робота специально выбран локальный ROS, а управление и видео уже доступны по HTTP. Для просмотра состояния Pi используйте SSH-раздел 7; для изображения — страницу в браузере Windows. Передача DDS между WSL и Pi — отдельная конфигурация сети, multicast и firewall, и в базовый запуск не входит.

## Официальные источники

- [ROS 2 Jazzy: Ubuntu deb installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html); [официальное зеркало](https://repo.test.ros2.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- [Пакет ros2-apt-source](https://github.com/ros-infrastructure/ros-apt-source).
- [Canonical: установка Ubuntu на Raspberry Pi](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/).
- [Netplan: примеры настройки сети](https://netplan.readthedocs.io/en/stable/examples/).
- [Microsoft: установка WSL](https://learn.microsoft.com/en-us/windows/wsl/install).
- [Microsoft: Linux GUI / WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).
- [OpenCV: свойства VideoCapture и ограничения backend](https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html).
- [tmux: официальный Getting Started](https://github.com/tmux/tmux/wiki/Getting-Started).
