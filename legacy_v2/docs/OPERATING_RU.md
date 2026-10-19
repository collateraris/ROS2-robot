# Запуск и работа с ROS 2 room world model v2

V2 использует типизированные ROS сообщения, действие `ApproachPerson` и отдельное разрешение оператора на движение. В поставленной конфигурации `allow_actuation: false`: Pi записывает камеру, а ПК проверяет perception и планирование в `shadow`. Камера, привод и world model в архиве не имеют готового физического допуска.

Руководство описывает реализованные команды и состояния. Сборка `colcon`, обмен DDS между Pi и ПК, Tk на целевом рабочем столе и работа настоящего робота в среде подготовки архива не проверялись. Результаты программных проверок приведены в [TESTING_RU.md](TESTING_RU.md), измерения камеры и данных — в [DATA_AND_CALIBRATION_RU.md](DATA_AND_CALIBRATION_RU.md), привода — в [HARDWARE_AND_SAFETY_RU.md](HARDWARE_AND_SAFETY_RU.md).

## 1. Распределение работы

| Устройство | Процесс | Назначение |
|---|---|---|
| Raspberry Pi | `room_world_model.ros_pi` | Единоличное владение камерой и serial, журнал, проверка свежести и геометрии, разрешение одного PWM-импульса |
| ПК | `room_world_model.ros_pc` | RGB perception, состояние world model, выбор действия и action server |
| Рабочий стол оператора | `room_world_model.operator` | Цель/направление, подтверждение неподвижности, ARM, удержание deadman и STOP |
| Терминал оператора | `room_world_model.goal_client` | Отправка `ApproachPerson`, feedback, результат и отмена |
| Arduino | `firmware/roomwm_drive/roomwm_drive.ino` | Протокол pulse v3, ограничение PWM/времени, подтверждение начала и выключения PWM |

Старые motor/web/serial процессы `robot_bluetooth_ros2` должны быть остановлены до открытия того же Arduino. Новая нода использует отдельный serial-протокол, не публикует движение в `/robot/cmd_vel` и не откатывается автоматически к старой прошивке. Диагностический `/roomwm/executed` не подключают к моторному мосту. Распиновка и ввод привода в эксплуатацию описаны в аппаратном руководстве.

Все имена `/roomwm/...` глобальные. Эта поставка предполагает одного робота и один активный PC controller. Для нескольких роботов нужны согласованные namespaces/remappings и отдельная проверка маршрутизации.

## 2. Окружение и обязательная сборка

### 2.1. Подготовка на каждой машине

Целевой вариант — ROS 2 Jazzy на Ubuntu 24.04: `arm64` на Pi и `amd64` на ПК. Настройте apt-репозиторий по [официальной инструкции Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html); [исходник инструкции](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst) также перечисляет эти платформы. После настройки репозитория:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rosidl-default-generators python3-venv python3-pip
```

На машине с окном оператора дополнительно нужны Tk и графический рабочий стол:

```bash
sudo apt install python3-tk
```

Распакуйте проект отдельно на Pi и ПК. Дальше текущий каталог — корень проекта с `setup.py`, `config` и `scripts`. На каждой машине создайте своё окружение системным Python, совместимым с установленным `rclpy`:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

`--system-site-packages` позволяет видеть системные ROS Python-пакеты. Готовая `.venv` не переносится между архитектурами.

На Pi:

```bash
python -m pip install -e '.[pi]'
```

На ПК сначала установите PyTorch для своего драйвера GPU по [официальному селектору](https://pytorch.org/get-started/locally/), затем зависимости проекта:

```bash
python -m pip install -e '.[ml,test]'
python -c 'import torch; print(torch.__version__); print("CUDA available:", torch.cuda.is_available())'
```

`CUDA available: True` проверяет доступность GPU именно этому Python. Полное время perception/planning измеряется отдельно. Для пассивной диагностики можно передать PC `--device cpu`; выполнение рабочего лимита `max_inference_s` на CPU не предполагается. Pi и ML extras рассчитаны на разные машины; избегайте одновременной установки GUI и headless pip OpenCV в одно окружение.

### 2.2. Генерация ROS interfaces

В v2 `roomwm_msgs` обязателен даже при запуске через `python -m`. `pip install -e` не генерирует сообщения. В активированном окружении **на Pi и на ПК**:

```bash
bash scripts/build_ros.sh
source scripts/ros_env.sh
python -c 'import rclpy; from roomwm_msgs.msg import FramePacket; from roomwm_msgs.action import ApproachPerson; print("ROS interfaces import OK")'
```

[build_ros.sh](../scripts/build_ros.sh) вызывает `colcon_core.command.main` выбранным `python3`, передаёт явные `--paths` к корневому `ament_python` пакету и `ros2/roomwm_msgs`, затем собирает оба. Явные пути нужны из-за вложенного интерфейсного пакета. Выходные каталоги — `ros_build`, `ros_install`, `ros_log`; используется `--symlink-install`.

В каждом новом терминале нод и ROS CLI:

```bash
source .venv/bin/activate
source scripts/ros_env.sh
```

[ros_env.sh](../scripts/ros_env.sh) загружает Jazzy и overlay, задаёт `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, снимает `ROS_LOCALHOST_ONLY`. Другой domain задавайте одинаково на обеих машинах **после** этого `source`.

Скрипт не настраивает маршрут, firewall, VPN или аутентификацию. На Windows/WSL2 PC-команды выполняются в Linux-окружении с доступным GPU; DDS-связь с Pi проверяется отдельно. Успешный `ping` и доступный GPU не подтверждают доставку ROS topics. При смене сетевых условий заново проверьте оба направления обмена до работы с приводом.

## 3. Первый запуск в shadow

### Pi: камера и запись

В `config/robot.yaml` оставьте `runtime.allow_actuation`, `runtime.allow_camera_only_research`, `runtime.calibration_confirmed`, `runtime.allow_wm_trials`, `runtime.world_model_validated` выключенными. У `camera.timing.accepted` и геометрической калибровки исходно тоже нет принятия. Эти флаги меняют после соответствующих измерений и проверки результатов.

До запуска задайте осмысленные `runtime.record_session` и `session.*` из раздела 8, проверьте настоящий путь камеры. Пробу выполняйте при остановленном Pi-процессе, поскольку у камеры должен быть один владелец:

```bash
python -m room_world_model.calibrate v4l2-probe \
  --config config/robot.yaml --frames 120 --warmup 5 \
  --output calibration/v4l2_probe.json
```

Затем:

```bash
python -m room_world_model.ros_pi --config config/robot.yaml
```

При `allow_actuation=false` serial-порт не открывается. Запись создаётся автоматически; каталог печатается в журнале. Не принятые по времени кадры всё равно доступны для диагностики с `timing_valid=false`, `settled=false`. Они не разрешают живой импульс и не становятся подтверждёнными обучающими переходами.

### ПК: модели и обработка

Один раз подготовьте локальные модели perception:

```bash
python -m room_world_model.download_models --config config/robot.yaml
```

Команда загружает настроенные person detector и metric indoor depth. Она не обучает WM и не выставляет допуски. Рабочая настройка `perception.local_files_only=true` требует готовые локальные файлы/кэш. Для воспроизводимых записей закрепите depth revision и согласуйте идентичность perception-моделей на Pi и ПК; правила fingerprint описаны в руководстве по данным.

В отдельном терминале ПК:

```bash
python -m room_world_model.ros_pc --config config/robot.yaml --mode shadow
```

Этот вариант использует baseline perception и не требует WM checkpoint. Для исследования уже созданной совместимой модели:

```bash
python -m room_world_model.ros_pc --config config/robot.yaml \
  --mode shadow --shadow-controller wm --checkpoint runs/room_v2/best.pt
```

Окно оператора запускается третьим процессом:

```bash
python -m room_world_model.operator
```

В shadow можно отправить задачу для проверки разбора, feedback и нулевых предложений. ARM в конфигурации без привода будет отклонён. При исходных шаблонах ожидаемы `calibration_required`, неизвестный возраст и недоверенная геометрия.

### Проверка графа

После старта Pi, затем PC:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /roomwm/frame --verbose
ros2 topic echo /roomwm/status --once
ros2 topic echo /roomwm/frame_info --qos-reliability best_effort --once
ros2 action list -t
ros2 action info /roomwm/approach_person
```

На ПК должны обнаруживаться Pi-публикации, а у Pi — PC-подписчик `/roomwm/frame`. После обработки кадров доступен `/roomwm/pc_status`. Для JPEG используйте `/roomwm/camera/image` и просмотрщик `sensor_msgs/CompressedImage` с совместимым best-effort QoS. Частота картинок не доказывает оптическую свежесть; её проверяют по exposure bounds и измерениям камеры.

### Запуск через launch

После той же сборки и подготовки окружения Pi:

```bash
ros2 launch room_world_model pi.launch.py \
  config:="$(pwd)/config/robot.yaml"
```

ПК:

```bash
ros2 launch room_world_model pc.launch.py \
  config:="$(pwd)/config/robot.yaml" mode:=shadow shadow_controller:=baseline
```

`config` — абсолютный путь. Связанные относительные пути считаются от корня проекта, содержащего каталог `config`. Launch не включает ARM/deadman. В PC launch нет отдельного trial-аргумента: для него используйте Python CLI ниже. Не запускайте launch и эквивалентную `python -m` ноду одновременно.

## 4. Режимы ПК и допуск WM

| Режим | Аргументы PC | Поведение и дополнительные требования |
|---|---|---|
| `shadow` | `--mode shadow` | Движение обнуляется; perception/диагностика продолжаются. `--shadow-controller`: `baseline`, `teleop` или `wm` |
| `teleop` | `--mode teleop` | Ручные `forward`, `left`, `right`; каждое движение проходит геометрию и Pi governor. `ApproachPerson` отклоняется |
| `baseline` | `--mode baseline` | Реакция на единственного видимого человека и текущую геометрию; WM checkpoint не нужен |
| Контролируемый WM trial | `--mode wm --wm-trial` | Proposal `controller=wm_trial`; реальные held-out action-ablation результаты, рассмотренный trial evidence, точный hash весов, `allow_wm_trials=true` |
| Принятый `wm` | `--mode wm` | Дополнительно рассмотренные парные физические WM/baseline испытания, acceptance evidence, `world_model_validated=true` |

`wm_trial` — значение поля proposal, не вариант `--mode` или `runtime.mode`. `--wm-trial` допустим только с `--mode wm`. На обеих ступенях WM проверяются пригодность checkpoint для реальных данных, calibration fingerprint и `runtime.world_model_sha256`. Pi читает review-файлы при старте и проверяет hash ненулевых WM proposals.

Файлы задают `runtime.world_model_trial_file` и `runtime.world_model_acceptance_file`. Схемы и критерии приведены в [WORLD_MODEL_RU.md](WORLD_MODEL_RU.md). Synthetic smoke checkpoint можно исследовать в shadow; его происхождение не допускает физический WM trial. Наличие `.pt` и успешное обучение не включают флаги автоматически.

Уже подготовленный supervised trial:

```bash
python -m room_world_model.ros_pc --config config/robot.yaml \
  --mode wm --wm-trial --checkpoint runs/room_v2/best.pt
```

Уже принятая модель:

```bash
python -m room_world_model.ros_pc --config config/robot.yaml \
  --mode wm --checkpoint runs/room_v2/best.pt
```

`runtime.mode` задаёт режим ПК по умолчанию, `--mode` его переопределяет. На Pi открытие motor serial независимо задаёт `runtime.allow_actuation`. Пассивность первого запуска обеспечивается именно `allow_actuation=false`. Все режимы с движением сохраняют калибровку, явный camera-only research opt-in, ARM, deadman, свежую геометрию и MCU pulse cap.

## 5. Типизированные интерфейсы и QoS

Определения — в [ros2/roomwm_msgs](../ros2/roomwm_msgs). ROS schema v2 и serial-протокол MCU v3 имеют разные номера версий. В таблице `BE/1` означает best effort, `KEEP_LAST`, depth 1, volatile; `R/1` — reliable, `KEEP_LAST`, depth 1, volatile. Реальные профили задают [ros_pi.py](../room_world_model/ros_pi.py) и [ros_pc.py](../room_world_model/ros_pc.py).

| Topic / service / action | ROS-тип | Направление | Профиль / назначение |
|---|---|---|---|
| `/roomwm/frame` | `roomwm_msgs/msg/FramePacket` | Pi → PC | BE/1; атомарный управляющий кадр |
| `/roomwm/frame_info` | `roomwm_msgs/msg/FrameInfo` | Pi → наблюдатель | BE/1; метки, возраст, происхождение |
| `/roomwm/camera/image` | `sensor_msgs/msg/CompressedImage` | Pi → наблюдатель | BE/1; просмотр JPEG |
| `/roomwm/actuation_proposal` | `roomwm_msgs/msg/ActuationProposal` | PC → Pi | R/1; lifespan = `safety.max_response_delay_s` |
| `/roomwm/actuation_ack` | `roomwm_msgs/msg/ActuationAck` | Pi → PC | Reliable, depth 64; MCU события/интервалы |
| `/roomwm/perception` | `roomwm_msgs/msg/PerceptionState` | PC → наблюдатель | BE/1; человек и текущая геометрия |
| `/roomwm/status` | `roomwm_msgs/msg/SafetyStatus` | Pi → PC/оператор | R/1; разрешение, причина, кадр, MCU |
| `/roomwm/pc_status` | `roomwm_msgs/msg/PlannerStatus` | PC → оператор | R/1; причина и время обработки |
| `/roomwm/task` | `roomwm_msgs/msg/TaskRequest` | PC action server / оператор → Pi и PC | R/1; новый `goal_id`, текст/направление |
| `/roomwm/deadman` | `std_msgs/msg/Bool` | Оператор → Pi | R/1; удержание разрешения |
| `/roomwm/emergency_stop` | `roomwm_msgs/msg/Stop` | Оператор / PC → Pi и PC | R/1; STOP, причина, необязательный `goal_id` |
| `/roomwm/executed` | `geometry_msgs/msg/TwistStamped` | Pi → наблюдатель | R/1; номинальное задание, не измеренная скорость/одометрия |
| `/roomwm/arm` | `std_srvs/srv/SetBool` | Оператор → Pi | `true`: явный ARM; `false`: STOP/disarm |
| `/roomwm/new_episode` | `std_srvs/srv/Trigger` | Оператор → Pi | Закрыть запись и открыть следующий episode |
| `/roomwm/approach_person` | `roomwm_msgs/action/ApproachPerson` | Клиент ↔ PC server | Goal, feedback, result, cancel; разрешение движения отдельно |

Профили внутренних service/action каналов остаются стандартными для используемого `rclpy`. Отдельная DDS deadline policy в нодах не задаётся. Lifespan есть у proposals; собственные временные проверки приложения действуют независимо от middleware.

### Атомарность и время кадра

`FramePacket` содержит `FrameInfo info`, `CompressedImage image`, до 256 `ActuationInterval history` и `history_start_s`. JPEG передаётся байтами без JSON/base64 в живом ROS-тракте. PC обрабатывает единый пакет. Отдельные diagnostic topics `frame_info` и `camera/image` могут теряться независимо и не используются для сборки управляющего кадра.

`FrameRef` содержит Pi session, camera epoch и sequence. `FrameInfo` хранит `pi_dequeue_monotonic_ns`, `exposure_earliest_ns`, `exposure_latest_ns`, время публикации на Pi, V4L2 timestamp/flags/sequence, `timestamp_source`, `timestamp_clock`, `timing_valid`, причину и ID evidence. Нулевые uint-поля при `timing_valid=false` или `has_v4l2_metadata=false` не обозначают измеренный timestamp; отсутствие восстанавливается из флагов.

`Header.stamp` — оценка в ROS/system time для инструментов. Движение проверяется по исходному интервалу Pi monotonic time в собственном ledger Pi. Часы ПК не вычитаются из часов Pi. Читайте `frame_age_valid` вместе с `frame_age_s`, `timing_valid` — вместе с exposure bounds. Обе реальные ноды запрещают `use_sim_time=true`.

BE/1 и последние доступные наблюдения уменьшают очередь, но не гарантируют максимальное время доставки. Reliable не означает физическое выполнение. Подтверждённый PWM-интервал требует соответствующего MCU результата; физическая остановка корпуса дополнительно требует торможения и settling.

## 6. Goal, ARM, deadman и отмена

### 6.1. Отправить задачу

Текст разбирает конечный словарь [commands.py](../room_world_model/commands.py). Неизвестный текст, отрицания и неоднозначные сочетания приводят к STOP. «Ко мне» не идентифицирует автора. Для движения нужен единственный текущий видимый человек с пригодным расстоянием; потеря или скачок цели прекращает задачу.

Откройте окно оператора, затем в отдельном терминале:

```bash
python -m room_world_model.goal_client \
  "подъедь к человеку и обходи препятствия" --standoff 1.0 --timeout 120
```

`--standoff` обязан совпадать с `perception.person_standoff_m` на PC; `1.0` подходит исходному значению. Goal не меняет скорость или запас и не отключает обход: `avoid_obstacles=false` отклоняется. Timeout положительный и не больше `safety.task_timeout_s` (900 секунд в примере). Новая action-цель при уже активном/reserved action отклоняется; сначала завершите или отмените прежнюю.

Клиент ждёт server до 5 секунд. Принятый goal становится `TaskRequest` с `goal_id`, равным UUID action, и снимает прежнее разрешение Pi. Он не выполняет ARM и не публикует `deadman=true`.

```bash
ros2 interface show roomwm_msgs/action/ApproachPerson
```

Goal: `standoff_m`, `timeout_s`, `avoid_obstacles`. Feedback: `phase`, `person_visible`, `person_distance_m`, `person_bearing_rad`, `completed_pulses`. Result: `success`, `reason`, `completed_pulses`. Счётчик увеличивается только по завершённым подтверждённым MCU-интервалам того же goal, с устранением повторов по `(boot_id, nonce, seq)`. Он не измеряет путь.

### 6.2. Разрешить откалиброванный опыт

Для уже подготовленного испытания с разрешённым приводом:

1. Запустите Pi и выбранный PC controller. Дождитесь принятия задачи, доступности камеры/MCU и пригодной геометрии первого действия.
2. В GUI визуально проверьте полную остановку колёс и место первого импульса, затем установите соответствующую галочку.
3. Переведите фокус из поля текста в другую часть окна. Удерживайте пробел и, не отпуская его, нажмите ARM.
4. Проверяйте `armed` и `reason` в Pi status. Продолжайте удержание, пока непосредственно наблюдаете допустимый ход опыта.
5. Для завершения отпустите удержание либо нажмите STOP / Escape. После снятия разрешения нужен новый явный ARM; завершившийся action требует нового goal.

**После отправки action не нажимайте «Отправить текстовую команду» в GUI повторно.** Это создаст другой `goal_id` и завершит прежний action с `superseded_by_new_task`. Здесь GUI используется для проверки, удержания и ARM уже принятой цели.

Можно удерживать кнопку разрешения мышью, но уход указателя с неё снимает mouse hold. Для одновременного ARM удобнее пробел вне поля текста. Пробел в поле вводит текст. Отпускание всех источников удержания публикует `deadman=false`; потеря фокуса дополнительно отправляет STOP. Закрытие GUI, Escape, STOP и потеря Pi status более чем на секунду также останавливают локальную задачу оператора. Heartbeat номинально публикуется каждые 50 мс; период под нагрузкой ОС не гарантирован.

ARM отклоняется при выключенном приводе, отсутствующей цели, незавершённом MCU STOP или невыполненных условиях governor. Если нужен новый zero handshake, явный запрос может ждать максимум 3 секунды. `success=true` сервиса в этом случае означает принятие запроса, а не установленное `armed=true`. Смотрите `SafetyStatus.arm_pending` и `armed`. Исходный дедлайн не продлевается; STOP, новый goal и `deadman=false` отменяют ожидание.

Штатное завершение импульса сохраняет ARM для следующего нового наблюдения той же цели при удержании. Ошибка, STOP, смена цели или потеря разрешения снимают ARM. Reconnect, новый кадр и восстановление сети его не возвращают. Прямой `/roomwm/arm` остаётся интерфейсом доверенного оператора; checkbox GUI не является сетевой аутентификацией.

### 6.3. Отмена и результат

В поставленном goal-клиенте `Ctrl+C` сначала публикует `/roomwm/emergency_stop`, затем запрашивает cancel принятого action и ограниченно ждёт ответ. Server при cancel тоже выдаёт STOP до ожидания результата отмены. Cancel/status не ждут завершения CUDA; старый GPU-результат с отменённым goal отбрасывается.

Достижение configured standoff даёт `success=true`, `reason=goal_reached_stop` и STOP. Timeout, потеря Pi status, неоднозначность/потеря человека, отсутствие прогресса и снятие ранее действовавшего ARM завершают action с причиной. При explicit action cancel сервер использует CANCELED; независимый STOP может завершить action как ABORTED с причиной STOP. После любого завершения старая цель не возобновляется.

Result подтверждает завершение программной задачи и выдачу STOP. Он не подтверждает неподвижность корпуса: дождитесь MCU PWM-off, расчётного settling и проверьте реальные колёса. Удержание не восстанавливается автоматически при возврате focus из терминала в GUI.

Отдельный STOP:

```bash
python -m room_world_model.goal_client "стоп"
```

Одноразовый клиент ждёт хотя бы одного STOP-подписчика до 2 секунд. При отсутствии завершится с ошибкой; наличие подписчика не является подтверждением MCU. Для ручной ROS-диагностики:

```bash
ros2 topic pub --once /roomwm/emergency_stop roomwm_msgs/msg/Stop \
  "{goal_id: '', reason: 'operator_cli_stop'}"
```

Пустой `goal_id` — общий STOP, непустой применяется к соответствующей задаче. Сетевой STOP зависит от доставки ROS; физическое отключение питания моторов остаётся отдельным способом прекратить привод.

### 6.4. Ручные демонстрации

Остановите прежний PC controller, затем:

```bash
python -m room_world_model.ros_pc --config config/robot.yaml --mode teleop
```

В GUI выберите «Вперёд», «Дуга влево» или «Дуга вправо», затем проверку, удержание и ARM. Каждое направление создаёт новый goal и снимает прежнее разрешение. Teleop сохраняет геометрические ограничения; неизвестный будущий footprint не разрешает движение. Задний ход и разворот на месте не входят в набор. `ApproachPerson` в teleop отклоняется.

## 7. Цикл движения и состояния Pi

1. Камера постоянно освобождает готовые V4L2-буферы; декодирование/публикация используют последний кадр. Timestamp/flags берутся из `DQBUF`, exposure bounds — из принятой native или EMPIRICAL калибровки. Длительность `read()` не подставляется вместо экспозиции.
2. Pi регистрирует исходные границы в своём ledger. Новый управляющий кадр пригоден, если **самая ранняя** возможная экспозиция не раньше конца расчётного торможения/settling. Поздняя публикация не освежает старое наблюдение.
3. PC worker обрабатывает один кадр, очередь заменяется последним наблюдением. WM согласует наблюдение с подтверждёнными кусочными действиями между exposure intervals. Пробелы истории сбрасывают/прогревают контекст, не заполняются выдуманными нулевыми действиями.
4. Perception проверяет текущую metric geometry всего будущего footprint конкретной дуги. MPC исследует будущие шаги, но предлагает только первый импульс.
5. Pi сверяет кадр, goal, lease, sequence, сроки/возраст, ту же геометрию/дугу, ограничения команды, ARM/deadman и MCU. Один кадр не разрешает повторный ненулевой импульс.
6. MCU ограничивает PWM-импульс и сообщает его границы. Pi ждёт подтверждённое PWM-off, торможение, settling и новое пригодное наблюдение.

Реализация находится в `ros_pi`, `SafetyGovernor`, `PulseSerial`, `ControlEngine` и адаптерах кадров. Подробные правила времени/действия приведены в руководстве по данным.

| `SafetyStatus.state` | Значение |
|---|---|
| `DISARMED` | Нет действующего разрешения; `arm_pending` отдельно показывает ожидание явного запроса |
| `PULSE_ACTIVE` | Governor выдал движение или есть незавершённый MCU импульс |
| `WAIT_SETTLE` | ARM сохранён, ожидается расчётное торможение/успокоение |
| `ARMED_STOPPED` | ARM сохранён и расчётный срок прошёл; новый импульс требует свежего кадра и проверок |

`motor_ready=true` — состояние serial-протокола, не измеренная неподвижность корпуса. Причина — `reason`, разрешение — `armed`, свежесть — `frame_age_valid` / `frame_age_s`. `settle_until_mono_s` относится к Pi и не вычитается из часов ПК.

Фронтальная камера может не видеть ближнюю боковую/заднюю часть объёма будущего корпуса. Свободный центр изображения не устраняет неизвестность. Если полный footprint не наблюдается, дуга остаётся запрещённой; уменьшение радиуса или запаса в YAML без измерения не создаёт наблюдение.

## 8. Сессии и `new_episode`

До независимой записи согласованно задайте путь и ID, например:

```yaml
runtime:
  record_session: sessions/room_day2_layout_b
session:
  session_id: room_day2_layout_b
  layout_id: layout_b
  day_id: day2
  lighting_id: evening_lamp
  operator: iulian
```

Это фрагмент полной конфигурации. Manifest сессии неизменяем: изменение robot/camera/config provenance требует другого `record_session`. Эти ID описывают условия эксперимента. Pi session в `FrameRef` и `SafetyStatus.session_id` — отдельный технический ID перезапуска.

Pi сохраняет `session_manifest.json`, отдельный episode с `metadata.json`, `events.jsonl`, исходными RGB JPEG. В журнале остаются goals, proposals, MCU события, timing, ошибки и границы. Только корректно закрытая запись получает `log_complete=true`.

Для разделения двух испытаний при неподвижном роботе завершите/cancel задачу, снимите ARM, дождитесь MCU PWM-off и физической остановки, затем:

```bash
ros2 service call /roomwm/new_episode std_srvs/srv/Trigger '{}'
```

Сервис отклоняет смену, пока Pi armed или есть незавершённый pulse. При успехе завершает старый writer, сразу открывает новый episode той же сессии, очищает историю и цель Pi. После границы задайте новую цель и выполните ARM. Сервис не меняет session ID/калибровку и не приостанавливает запись.

**Для ручного переноса сначала штатно завершите Pi-процесс/запись после STOP и полной остановки, затем перенесите робот и запустите Pi снова.** `new_episode` пишет непрерывно; вызов только после переноса не превращает предыдущие кадры переноса в известное нулевое действие. Ручное движение не должно попадать внутрь подтверждённого обучающего перехода. При новой расстановке/условиях смените `record_session` и ID до возобновления записи.

Экспорт закрытых эпизодов:

```bash
python -m room_world_model.record \
  --session sessions/room_day2_layout_b --output data/exported
```

Нужны принятые exposure intervals и полное MCU-покрытие действий. Пустой PWM-список без подтверждённого exclusive zero handshake не доказывает нулевое действие. Unknown outcome, reboot, gap и episode boundary обрывают допустимые переходы. `camera.max_training_exposure_uncertainty_s=0.10` ограничивает **половину ширины** exposure interval; более широкий переход отклоняется, даже если кадр полезен для просмотра. Сохраняйте исходные журналы и RGB вместе с экспортом. Разметка, masks, grouping и проверка v1-данных описаны в [DATA_AND_CALIBRATION_RU.md](DATA_AND_CALIBRATION_RU.md).

## 9. Параметры и диагностика

Числа взяты из [config/robot.yaml](../config/robot.yaml); это параметры примера, не результаты измерения вашей машины.

| Параметр | Значение | Смысл |
|---|---:|---|
| `camera.requested_fps` / `publish_max_fps` | 15 / 10 | Запрошенный V4L2 поток / верхняя частота публикации |
| `camera.timing.accepted` | `false` | Без принятого timing нет допуска по свежести |
| `camera.max_capture_age_at_dequeue_s` | 1.00 с | Верхняя граница возраста уже при получении кадра Pi |
| `camera.max_training_exposure_uncertainty_s` | 0.10 с | Полуширина интервала для WM переходов |
| `safety.pulse_s` | 0.08 с | Планируемый моторный импульс |
| `robot.linear_command_scale_mps` | 0.06 м/с | Верхнее номинальное задание вперёд |
| `robot.physical_speed_bound_mps` | 0.10 м/с | Физическая граница для тормозного расчёта |
| `safety.max_response_delay_s` | 0.35 с | Pi dequeue → возврат proposal; также DDS lifespan proposals |
| `safety.max_inference_s` | 0.30 с | PC worker queue + decode + perception + planning |
| `safety.max_frame_age_s` | 2.00 с | Возраст от earliest exposure по Pi clock |
| `safety.deadman_timeout_s` | 0.35 с | Давность действующего удержания |
| `safety.settle_extra_s` | 0.20 с | Дополнительное успокоение после торможения |
| `safety.clearance_margin_m` | 0.25 м | Добавочный пространственный запас |
| `perception.person_standoff_m` | 1.00 м | От модели центра корпуса до видимой поверхности человека |
| `planner.wait_s` | 2.50 с | Пауза в rollout; фактически нужен новый пригодный кадр |

`0.06 × 0.08 = 0.0048 м` — произведение номинальных параметров, не измеренный путь. PWM, мёртвая зона, разгон, батарея, поверхность и выбег меняют движение. В v2 нет принятого camera latency bound по умолчанию: его получают измерением, не длительностью чтения.

| Причина / симптом | Что проверить |
|---|---|
| `No module named roomwm_msgs` | Сборку обоих пакетов и source overlay тем же Python |
| Нет frame/action server | Pi/PC, одинаковый domain, DDS-маршрут, QoS; action создаётся после загрузки PC-моделей |
| `No STOP subscriber discovered` | Клиент не обнаружил Pi/PC-подписчика; не считать STOP доставленным |
| `timing_valid=false`, неизвестный возраст | DQBUF flags/clock, timing evidence, режим камеры/controls |
| `calibration_required` | Подтверждение физической калибровки runtime |
| `camera_only_research_opt_in_required` | Явный допуск ограниченного camera-only опыта |
| `geometry_untrusted_or_unobserved`, `no_currently_observed_safe_arc` | Metric depth, rectification, valid mask, наблюдение полного footprint |
| `insufficient_braking_clearance` | Тормозной расчёт и реальную свободную дистанцию |
| `waiting_for_post_settle_frame` | Штатное ожидание поздней экспозиции после остановки |
| `world_model_history_warmup` | Принятую историю наблюдений и подтверждённых действий для burn-in |
| `deadman_stale` | Удержание, focus GUI, operator и доставку heartbeat |
| `drive_ready_stale` | Serial, прошивку, свежие ответы MCU, zero handshake |
| `camera_stale` | Исходные exposure bounds, USB/очередь и camera stall |
| `stale_inference_result` | Полный Pi dequeue → PC → Pi; depth 1 не устраняет все очереди |
| `pc_decode_perception_planning_deadline_exceeded` | Полное время PC, GPU, конкурирующую нагрузку |
| `wrong_or_expired_lease`, `reused_inference_frame` | Старые/дублирующиеся процессы, смену цели/камеры, новый кадр |
| `target_jump_stop_reselect`, потеря/несколько людей | Однозначность цели; остановиться и выбрать задачу заново |
| `no_progress_stop` | Реальный ход, PWM dead zone, проходимость, distance; не добавлять скрытый PWM boost |
| `superseded_by_new_task` | Новый TaskRequest заменил action; не дублировать цель GUI-кнопкой |
| WM hash/evidence mismatch | Точный checkpoint, реальные review-данные, fingerprint, ступень допуска |
| `recorder_overflow_or_disk_error`, `log_complete=false` | Диск и очередь; флаг полного журнала не исправляется вручную |

Для наблюдения без выдачи разрешений используйте отдельные терминалы:

```bash
ros2 topic echo /roomwm/status
```

```bash
ros2 topic echo /roomwm/pc_status
```

После STOP сначала разберите причину. Перезапуск программы не измеряет камеру, не исправляет неизвестную геометрию и не подтверждает историю привода.

## 10. Границы готовности

В этой среде проверялись программные контракты и offline-тесты. Не выполнялись генерация типов в установленном Jazzy, реальный DDS round trip, аппаратный optical latency experiment, оценка RTX 4090 latency, физический stop-time тест или парные robot trials. Message-shaped fixtures в тестах не заменяют generated ROS interfaces.

Даже после ROS-сборки отдельно остаются измерения оптической задержки, неопределённости MCU clock, максимальной скорости/выбега, доставки STOP под нагрузкой и depth в комнате. Повтор старых пикселей с новыми правдоподобными метками не распознаётся надёжно одним протоколом. Фронтальная monocular camera не измеряет невидимые поверхности и не даёт encoder feedback.

Режим предназначен для контролируемого исследовательского опыта с непосредственным наблюдением оператора. Полезность WM определяется сравнением с baseline при одинаковых условиях и ограничениях, пригодность привода — измерениями на той же платформе. Команды руководства задают порядок такого опыта; его результаты ещё нужно получить.
