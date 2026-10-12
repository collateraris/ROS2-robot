# Установка на вашу Raspberry Pi и запуск с Android / Windows

Используются ваши данные: Ubuntu 24.04 arm64, ROS 2 Jazzy, пользователь `shapturov`, адрес Pi `192.168.0.28`, USB-камера `/dev/video0`. Нужна Uno R3 / классическая Nano ATmega328P. В браузере телефона будут джойстик, СТОП и изображение; существующее Bluetooth-приложение продолжит управлять через прежний модуль.

**Перед первой загрузкой прочитайте HARDWARE_RU.md: ENA нужно перенести с D1 на D9.** Новая папка называется `robot_bluetooth_ros2`; это отдельный профиль под ваши три файла. Старый `robot_wifi_teleop` не запускайте одновременно с новым: у них одинаковые имя ROS-пакета, порт 8080 и аппаратные устройства.

## 1. SSH доступен после включения Raspberry Pi

На Windows, PowerShell:

```powershell
ssh shapturov@192.168.0.28
```

После входа выполняйте следующие команды **на Pi**, в этом SSH-терминале:

```bash
if systemctl is-active --quiet ssh.socket; then
    sudo systemctl enable ssh.socket
else
    sudo systemctl enable ssh.service
fi
systemctl list-unit-files ssh.service ssh.socket
```

Команда сохраняет тот способ запуска, которым уже обслуживается ваш SSH. Для выбранного unit в столбце STATE должно быть `enabled`. В Ubuntu 24.04 SSH обычно запускается по входящему подключению через `ssh.socket`; при этом `ssh.service` может быть `disabled`, и это не означает, что SSH после загрузки недоступен. `enable` включает автозапуск, не останавливая текущую SSH-сессию.

Pi должна загрузиться и подключиться к известной Wi-Fi-сети. Для сохранения адреса `192.168.0.28` откройте настройки домашнего роутера и закрепите этот адрес за Wi-Fi MAC-адресом Pi в DHCP reservation / Address Reservation. Не назначайте случайный статический адрес через SSH: ошибка сетевой конфигурации может лишить доступа. Подключение с Windows всегда остаётся `ssh shapturov@192.168.0.28`.

[Ubuntu 24.04: SSH socket activation](https://documentation.ubuntu.com/release-notes/24.04/), [systemctl](https://manpages.debian.org/bookworm/systemd/systemctl.1.en.html), [DHCP reservation Raspberry Pi](https://www.raspberrypi.com/documentation/computers/configuration.html#manage-dhcp-and-static-ip).

## 2. Прошить Arduino и скопировать проект

Скачайте полный архив и распакуйте так, чтобы в Windows существовала папка:

```text
C:\Users\twoli\Downloads\robot_bluetooth_ros2
```

Внутри неё сразу должны быть `arduino`, `ros2_ws`, `scripts`, `docs`; Проводник иногда добавляет дополнительную внешнюю папку с именем архива. Не копируйте только внешнюю оболочку.

На Windows откройте `arduino/blu_drive/blu_drive.ino` в Arduino IDE. `motor.h` и `move_case.h` должны лежать рядом. Выберите вашу Uno/Nano и её COM-порт, скомпилируйте и загрузите. Сначала перенесите ENA на D9 и отключите питание моторов на время переподключения. После прошивки закройте Serial Monitor, подключите Arduino к Pi кабелем USB с передачей данных.

В отдельном **PowerShell Windows**:

```powershell
Test-Path "$env:USERPROFILE\Downloads\robot_bluetooth_ros2\scripts\run_robot.sh"
scp -r "$env:USERPROFILE\Downloads\robot_bluetooth_ros2" shapturov@192.168.0.28:~/
```

Первая команда должна вернуть `True`. Не используйте `127.0.0.1` или `127.0.1.1`: из PowerShell это адрес самого компьютера, не Pi.

## 3. Зависимости ROS 2

Поскольку вы уже установили Jazzy, повторная установка ОС и ROS не нужна. В SSH на **Pi**:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y ros-jazzy-cv-bridge python3-opencv python3-aiohttp python3-serial python3-yaml ros-dev-tools v4l-utils unzip tmux
sudo usermod -aG dialout,video shapturov
```

Выйдите командой `exit` и снова подключитесь с Windows, чтобы новая оболочка получила группы `dialout` и `video`:

```powershell
ssh shapturov@192.168.0.28
```

На Pi:

```bash
groups
ls -l /dev/serial/by-id/
v4l2-ctl --list-devices
```

Для Arduino используйте найденный `/dev/serial/by-id/...`, если он существует. У Uno это часто `/dev/ttyACM0`, у Nano с USB-UART адаптером — `/dev/ttyUSB0`; выбирайте фактически появившееся устройство. Проект не угадывает порт. В примере запуска ниже `/dev/ttyACM0` нужно заменить, если ваша плата определяется иначе.

### Если Jazzy на новой Pi ещё нет

Нужна именно Ubuntu 24.04 arm64 / Noble, не Raspberry Pi OS. Проверка:

```bash
cat /etc/os-release
uname -m
```

Для свежей Ubuntu 24.04 используйте включённый установщик вместо ручного добавления второго ROS-репозитория:

```bash
cd ~/robot_bluetooth_ros2
bash scripts/install_jazzy.sh pi
```

Он устанавливает ros-jazzy-ros-base, зависимости проекта и rosdep. После изменения групп снова войдите по SSH. [Официальная инструкция Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [официальное зеркало](https://repo.test.ros2.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### Если повторится ваша ошибка apt с liblz4 / libzstd

Сначала проверьте, что в `/etc/apt/sources.list.d/ubuntu.sources` у обычного архива Ubuntu включены `noble` и `noble-updates`. Ранее в присланном выводе был виден `noble-security`, но не `noble-updates`: это возможная причина несогласованных версий, а не доказательство held-пакетов. Добавьте `noble-updates` к существующей строке Suites обычного Ubuntu-архива, сохранив URIs, Components, Signed-By и отдельную секцию security; затем выполните `sudo apt update` и повторите установку. Не понижайте библиотеки и не удаляйте пакеты вслепую.

## 4. Сборка нового проекта

В **новом SSH-терминале Pi**, чтобы не наследовать вручную подключённую старую сборку:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_bluetooth_ros2/ros2_ws
```

Если rosdep уже инициализирован, повторять init не нужно. Для новой системы:

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix robot_web_teleop
```

Последний путь должен указывать на `robot_bluetooth_ros2/ros2_ws/install`, а не на старый проект. Если в `.bashrc` автоматически подключается старый workspace, подключите новый `install/setup.bash` последним. Не копируйте build/install/log с Windows или старого workspace.

Исправление `Cannot locate rosdep definition for [ament_python]` уже учтено: в package.xml сохранён `<export><build_type>ament_python</build_type></export>`, ошибочного `<buildtool_depend>ament_python</buildtool_depend>` нет.

## 5. Запуск робота и телефона

Остановите предыдущие `robot.launch.py`, `camera_node`, `web_node`, Serial Monitor и любые другие процессы, владеющие камерой или Arduino. В их терминале используйте `Ctrl+C`. Если работали в tmux, сначала вернитесь в тот сеанс; второй сервер на 8080 запускать нельзя.

Первое испытание — колёса подняты. На **Pi**:

```bash
cd ~/robot_bluetooth_ros2
bash scripts/run_robot.sh serial_port:=/dev/ttyACM0 camera_device:=/dev/video0 camera_required:=false
```

Если Arduino — `/dev/ttyUSB0`, передайте `serial_port:=/dev/ttyUSB0`. Можно указать фактический длинный by-id путь. Скрипт запросит пароль страницы, минимум 8 символов. Это отдельный пароль пульта, не обязательно пароль SSH. Оставьте запуск работающим.

На **Android**:

1. Подключитесь к той же домашней Wi-Fi-сети, что и Pi. Сеть должна разрешать соединения между клиентами.
2. В Chrome или другом современном браузере откройте **http://192.168.0.28:8080/**.
3. Введите заданный пароль. При необходимости сделайте ярлык страницы через меню браузера; отдельный APK не требуется.
4. Дождитесь «Arduino: Wi-Fi готов», отпустите кнопки старого Bluetooth-пульта и нажмите «Взять управление».
5. Удерживайте джойстик для движения. Отпускание посылает ноль. Кнопка СТОП запрашивает остановку Wi-Fi и Bluetooth.

Для `camera_required:=false` интерфейс показывает «Управление в прямой видимости — смотри на робота». При ваших медленных кадрах этот профиль позволяет ручное управление с наблюдением за самим роботом. Остановка/отсутствие камеры не блокирует движение; связь с Arduino и свежие команды остаются обязательными.

Для режима, требующего свежего видео, остановите старый запуск и выполните:

```bash
bash scripts/run_robot.sh serial_port:=/dev/ttyACM0 camera_device:=/dev/video0 camera_required:=true
```

При чтении 0.6–0.75 с ваша камера может не пройти этот контроль. Причина низкой частоты ещё не установлена; увеличение разрешённого возраста не устраняет задержку реального изображения. См. CAMERA_SLOW_RU.md.

Чтобы проверить только изображение, остановите полный launch и запустите:

```bash
bash scripts/view_camera.sh camera_device:=/dev/video0
```

В этом режиме нет управления моторами и USB-мост не запускается. Возвращаясь к управлению, остановите просмотр через Ctrl+C.

### Сохранить процесс при закрытии SSH

Автозапуск SSH и работа ROS-программы — разные задачи. Для сохранения текущего запуска используйте tmux:

```bash
tmux new -s robot
cd ~/robot_bluetooth_ros2
bash scripts/run_robot.sh serial_port:=/dev/ttyACM0 camera_required:=false
```

Отсоединиться от tmux: `Ctrl+B`, отпустить клавиши, затем `D`. Вернуться: `tmux attach -t robot`. Для остановки программы вернитесь в сеанс и нажмите Ctrl+C. Само включение SSH не запускает моторы и не включает этот launch.

## 6. Контроль из терминала ROS

В другом SSH-сеансе Pi:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_bluetooth_ros2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list
ros2 topic echo /robot/drive/source
```

Чтобы смотреть готовность вместо источника, завершите echo и выполните:

```bash
ros2 topic echo /robot/drive/ready
```

Явный общий СТОП из терминала:

```bash
ros2 topic pub --once /robot/drive/stop std_msgs/msg/Empty '{}'
```

Обычный нулевой cmd_vel не предназначен для остановки Bluetooth. До/после BT используйте штатные кнопки и проверьте фактическое состояние робота. Подтверждения serial — это применённый PWM, а не датчик вращения.

Управление через браузер не требует DDS между Windows и Pi: ROS-узлы работают на Pi, сеть передаёт HTTP/WebSocket. Windows нужен для SSH, Arduino IDE и браузера. RTX 4090 для ручного пульта не используется. Если позже понадобится ROS на ПК, можно установить Ubuntu 24.04 в WSL2 и Jazzy отдельно; к этому запуску он не нужен.

## 7. Если страница не открывается

Проверьте, что launch ещё работает и не сообщил об ошибке порта/пароля. На Windows:

```powershell
Test-NetConnection 192.168.0.28 -Port 8080
```

На Pi:

```bash
ss -ltnp 'sport = :8080'
```

Если SSH работает, а 8080 — нет, сначала проверьте процесс web_node. Если он слушает, проверьте firewall и изоляцию Wi-Fi-клиентов. Открывайте нужный порт только для своей домашней сети; не отключайте весь firewall и не пробрасывайте 8080 в интернет. Если браузер показывает Bluetooth, сначала отпустите кнопку Bluetooth и дождитесь ready, затем заново возьмите управление.
