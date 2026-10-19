# Неделя 2. Объекты камеры и проверенная привязка к LiDAR

Цель недели — получить два разных результата: bounding boxes на изображении и,
только когда геометрии достаточно, координаты поддержанных лазерными точками
объектов в `map`. Для движения к объекту одного bounding box недостаточно.
На этой неделе код не публикует скорости, PWM или цели Nav2.

Сначала выполните приёмку недели 1: `/scan`, `/odom`, TF и навигация работают.
Затем проверяйте детектор на сохранённом кадре с неподвижным роботом. Для
экспериментов подходят штатная камера и RPLIDAR C1; энкодеры не добавляются.

## Что реализовано

| Путь | Назначение |
|---|---|
| `room_semantics/detector.py` | Реальный pretrained GroundingDINO через Transformers; загрузка весов по требованию |
| `room_semantics/detect_image.py` | CLI для вашего кадра: JSON и картинка с boxes |
| `room_semantics/calibration.py` | Проверка схемы калибровки, pinhole и optical/LiDAR преобразования |
| `room_semantics/geometry.py` | Проекция скана, группировка возвратов, отказы при неоднозначности |
| `room_semantics/tracking.py` | Простые стабильные ID по классу и расстоянию, срок жизни наблюдения |
| `ros2_ws/src/room_perception` | ROS-узел: камера, CameraInfo, LiDAR, TF → два диагностических топика |
| `weeks/week02` | Зависимости, пример запуска и отдельная калибровка симулятора |

Это камера + геометрия + pretrained detector. World model и tiny-VLA из
`learning_nav` остаются отдельными исследовательскими модулями; их обучение
для этой недели не требуется.

## Установка ML-окружения

Используйте Ubuntu 24.04 и ROS 2 Jazzy из общей инструкции установки. Виртуальное
окружение должно использовать системный Python 3.12 и видеть ROS-пакеты.
Для воспроизводимой проверки на CPU:

```bash
cd ~/robot/room_robot_v5_four_weeks
bash scripts/setup_python.sh perception cpu
source scripts/env_robot.sh
```

Если проект находится в другом каталоге, замените путь. Для GPU сначала
установите выбранную CUDA-сборку PyTorch в `.venv` по разделу 4
[INSTALL_RU.md](INSTALL_RU.md), затем выполните:

```bash
bash scripts/setup_python.sh perception existing
source scripts/env_robot.sh
python -c 'import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name())'
```

Дополнительная установка `requirements-perception.txt` после этих команд не
требуется. Если устанавливаете `torchvision` отдельно, его версия должна
соответствовать выбранному PyTorch и CUDA. На ПК используйте `--device cuda`; на Pi
или CPU скорость не обещается. При слишком долгом инференсе метрические
наблюдения будут отклоняться по возрасту.

В коде закреплён `transformers==4.52.3`. В этой версии используется
`post_process_grounded_object_detection(..., threshold=..., text_threshold=...)`
и строковые `text_labels`. Не заменяйте параметр на `box_threshold` из старых
примеров. Источник API:
[официальная документация 4.52.3](https://huggingface.co/docs/transformers/v4.52.3/model_doc/grounding-dino).
Имена классов — английские `person`, `chair`, `table`, `door`; русские команды
появляются в слое задач недели 3.

## Настоящий запуск pretrained detector на кадре

Положите ваш кадр комнаты в доступный файл и выполните:

```bash
bash weeks/week02/run_image_smoke.sh /absolute/path/to/room.jpg
```

Первый запуск загружает `IDEA-Research/grounding-dino-tiny` из Hugging Face.
Для кэшированных весов и контролируемого offline-запуска:

```bash
python -m room_semantics.detect_image /absolute/path/to/room.jpg \
  --out runs/week02/frame.json --annotated runs/week02/frame.jpg \
  --device cuda --local-files-only
```

В JSON записываются реальное время инференса, модель и, если доступен,
разрешённый commit весов. По умолчанию в CLI и ROS закреплён проверенный commit
`a2bb814dd30d776dcf7e30523b00659f4f141c71`; другую модель/revision задавайте
осознанно через `--model`, `--revision` и соответствующие ROS-параметры.
Пустой список detections — допустимый результат.
Распознавание примитивной фигуры «человека» в симуляторе не гарантируется и
не заменяет проверку на ваших изображениях.

## Калибровка: что именно нужно измерить

Для настоящей USB-камеры сначала выполните [CAMERA_RU.md](CAMERA_RU.md):
установка `usb_cam`, подбор video-режима, intrinsics шахматной доской,
`image_proc` и измеренные static TF. В симуляторе USB-драйвер не требуется.

Камера должна быть **pinhole без дисторсии или выдавать rectified image**. Для
обычной USB-камеры сначала получите intrinsics и коэффициенты дисторсии, затем
используйте исправленное изображение, например через ROS `image_proc`.
CameraInfo должен соответствовать этому изображению, разрешению и optical frame.
Узел использует матрицу `P` CameraInfo, не угадывает focal length и не заменяет
неизвестную глубину оценкой по размеру человека. После калибровки не меняйте zoom,
разрешение, crop или положение сенсоров без повторной проверки.

Скопируйте шаблоны:

```bash
mkdir -p config/week02
cp ros2_ws/src/room_perception/config/calibration.yaml config/week02/calibration.yaml
cp ros2_ws/src/room_perception/config/perception.yaml config/week02/perception.yaml
```

Пример структуры файла; числа ниже — геометрия симулятора, а не вашего робота:

```yaml
schema_version: 1
verified: false
image_is_rectified: false
camera_frame: camera_optical_frame
lidar_frame: laser
map_frame: map
fixed_frame: odom
camera_from_lidar:
  translation_m: [0.0, -0.02, -0.085]
  quaternion_xyzw: [0.5, -0.5, 0.5, 0.5]
tf_tolerance:
  translation_m: 0.02
  rotation_rad: 0.03
```

`camera_from_lidar` — преобразование **точки из `laser` в optical-систему камеры**:
`p_camera = R * p_lidar + t`. Optical axes: X вправо, Y вниз, Z вперёд.
Порядок quaternion: **x,y,z,w**. Измерьте положение/ориентацию сенсоров или
используйте процедуру extrinsic calibration; добавьте согласованные static TF
через описание робота. При неверном TF программный флаг не исправляет геометрию.

Проверка формата:

```bash
python -m room_semantics.validate_calibration config/week02/calibration.yaml
```

Это только проверка файла. Она **не измеряет** сенсоры. Проверьте наложение
спроецированных лазерных точек на реальные предметы на нескольких расстояниях и
по краям кадра. Только после этого задайте `verified: true`,
`image_is_rectified: true` в calibration и `image_is_rectified: true` в параметрах
узла. В параметрах укажите реальный rectified image topic. Узел сверяет
extrinsics из файла с TF; несовпадение блокирует метрические объекты. Для
симулятора есть отдельный `calibration.sim.yaml`; на роботе его не используйте.

## Live ROS

Пересоберите рабочую область с новым `room_perception` в **новом терминале без
активного venv/Conda**. Если используете прежний терминал, сначала выполните
`deactivate` для venv; Conda также должна быть выключена. Только после сборки
подключите окружение:

```bash
cd ~/robot/room_robot_v5_four_weeks
bash scripts/build_ros.sh week02
source scripts/env_robot.sh
```

Для аппаратного варианта без Gazebo вместо `week02` используйте `hardware` по
общей инструкции. Во всех новых терминалах из корня проекта выполняйте
`source scripts/env_robot.sh` перед командами ROS и Python.

Терминал с настоящими камерой, LiDAR и навигацией остаётся запущенным. Запуск
perception в другом терминале:

```bash
source scripts/env_robot.sh
ros2 launch room_perception perception.launch.py \
  python_executable:="$ROOM_PROJECT_ROOT/.venv/bin/python" \
  params_file:="$ROOM_PROJECT_ROOT/config/week02/perception.yaml" \
  calibration_file:="$ROOM_PROJECT_ROOT/config/week02/calibration.yaml"
```

Важно передать Python окружения: установленный `ros2 run` console script может
иметь shebang системного `/usr/bin/python3`, где нет Transformers. Launch
исполняет его указанным Python. Чистые `room_semantics` доступны через PYTHONPATH
из `env_robot.sh` и editable-установку общего проекта.

Вариант для недели 1 в симуляторе, при работающем SLAM/Nav2:

```bash
source scripts/env_robot.sh
ros2 launch room_perception perception.launch.py \
  python_executable:="$ROOM_PROJECT_ROOT/.venv/bin/python" \
  params_file:="$ROOM_PROJECT_ROOT/weeks/week02/perception.sim.yaml" \
  calibration_file:="$ROOM_PROJECT_ROOT/weeks/week02/calibration.sim.yaml" \
  use_sim_time:=true
```

Симуляторная камера — 640×480, 60° horizontal FOV, pinhole без дисторсии.
Её Image и CameraInfo имеют frame `camera_optical_frame`; скан — `laser`.
Сначала дождитесь свежего `/room_base/odom_healthy=true`. Ground truth из
`/sim/odom` или файла сцены perception не читает.

Для прямого запуска параметры поддерживаются стандартно:

```bash
"$ROOM_PROJECT_ROOT/.venv/bin/python" -m room_perception.node --ros-args \
  --params-file "$ROOM_PROJECT_ROOT/config/week02/perception.yaml" \
  -p calibration_file:="$ROOM_PROJECT_ROOT/config/week02/calibration.yaml" \
  -p image_topic:=/camera/image_rect \
  -p camera_info_topic:=/camera/camera_info -p scan_topic:=/scan
```

Прямой запуск также требует `source scripts/env_robot.sh`, чтобы overlay пакета
`room_perception` был в PYTHONPATH.

## Почему bbox + луч не всегда дают координату

Узел проецирует конечные валидные точки плоского LaserScan в изображение.
Нужен compact foreground cluster минимум из трёх соседних согласованных
возвратов внутри bbox. Передний кластер должен отделяться по глубине **с обеих
сторон** от более дальних валидных возвратов. Проверяются spread, разумная ширина
сечения и расположение относительно центра bbox. Несколько пригодных кластеров
означают отказ. Два пересекающихся semantic boxes не могут присвоить один
кластер двум объектам.

Поэтому стена, видимая внутри bbox человека, сама по себе не становится
«координатой человека». Человек возле стены, тонкие ножки стула, проём двери,
стекло, окклюзия или отсутствие дальних соседей часто дадут **только 2D detection**.
Это ожидаемый отказ. Эта эвристика не является доказательством принадлежности
лучей объекту; foreground может оказаться другим объектом. Для более общего
случая понадобятся segmentation/depth/3D LiDAR и отдельная оценка качества.

Результат — **метрические bearing/range и точка кластера в плоскости LiDAR**.
`position.z` — высота этой плоскости в `map`, а не высота центра человека.
Код явно ставит `localization: planar_lidar_cluster_not_3d_center` и ненулевую
`uncertainty_m`; она увеличивается с задержкой, особенно для людей.

## Время, очереди и потеря наблюдений

Camera header должен содержать время получения кадра сенсором, не момент
завершения нейросетевого инференса. Камера, Pi и ПК должны использовать
согласованные ROS clocks. Для Gazebo включайте `use_sim_time` у всех связанных
узлов. Простое назначение «сейчас» старому кадру нарушает проверку задержки.

- Worker для detector имеет один заменяемый pending frame; старые кадры не
  накапливаются в очереди за медленным инференсом.
- В выходе сохраняются `image_stamp_ns`, время получения кадра и завершения
  inference. Исходный CameraInfo/intrinsics берётся из snapshot кадра.
- Проекция использует `camera_optical_frame` **в момент кадра** и `laser`
  **в момент скана**, через `lookup_transform_full(..., fixed_frame='odom')`.
  Позиция в карте берётся из TF `map <- laser` в момент скана.
- Bbox можно повторно привязать к более новому скану только при разнице времени
  <=0.12 с; camera acquisition stamp при этом сохраняется. Возраст кадра
  ограничен 0.5 с, скана — 0.3 с. Иначе объекта для нового момента не добавляется.
- Свежий `odom_healthy` обязателен; false/просроченный статус очищает metric scene.
- ID сопоставляются по классу и расстоянию. Они стабильны при простом движении,
  но не являются идентификацией человека. При пересечении похожих объектов
  ассоциация может стать неоднозначной; planner обязан остановиться/уточнить.
- TTL отсчитывается от исходного скана: <=0.8 с. Повторная публикация того же
  скана не продлевает жизнь объекта. Таймер публикует сцену 5 Гц и удаляет
  просроченные объекты.

Робот и человек могут двигаться между кадром и сканом. Timestamped TF
компенсирует движение робота; движение человека не восстанавливается. Для
первых проверок останавливайте робота, затем наблюдайте сцену. Даже при
неподвижном роботе общая граница возраста кадра сохраняется.
Поточечная deskew-коррекция вращающегося скана не реализована: проекция использует
timestamp начала скана для всех лучей. Быстрые повороты и длинная развёртка
увеличивают ошибку; текущие проверки предназначены для медленного движения.

## Формат сообщений для недели 3

`/room_semantics/detections`, `std_msgs/String`, JSON: `schema_version`,
`stamp_ns=image_stamp_ns`, `image_received_ns`, `inference_finished_ns`,
`inference_seconds`, `frame_id`, размеры кадра, `detections` со строковым
`label`, `score`, `bbox_xyxy`. `metric_localization=false` всегда. Сообщение может
иметь `stale=true`; использовать его как метрическую цель нельзя.

`/room_semantics/objects`, `std_msgs/String`, JSON:

```json
{
  "schema_version": 1,
  "stamp_ns": 1000000000,
  "image_stamp_ns": 1000000000,
  "scan_stamp_ns": 1010000000,
  "frame_id": "map",
  "objects": [{
    "id": "person_0001",
    "label": "person",
    "score": 0.9,
    "position": {"x": 2.0, "y": 0.0, "z": 0.154},
    "uncertainty_m": 0.21,
    "source": "camera_lidar",
    "last_seen_ns": 1010000000,
    "localization": "planar_lidar_cluster_not_3d_center"
  }]
}
```

Это **пример схемы**, не реальный запуск. Объекты имеют дополнительные stamps,
range/bearing, support_points и bbox. `published_ns` — время публикации;
`stamp_ns` сохраняет источник наблюдения и не обновляется искусственно таймером.
`status`, `perception_status`, `rejections`, `calibration_verified` и hash
calibration помогают разбирать отказы. Отсутствие объектов — штатный результат,
после которого week03 не должен угадывать координаты из bbox или scene manifest.

## Приёмка и ограничения проверки

```bash
python -m pytest tests/test_semantics_geometry.py tests/test_semantics_detector.py -q
python -m room_semantics.validate_calibration weeks/week02/calibration.sim.yaml --require-verified
ros2 topic echo /room_semantics/detections
ros2 topic echo /room_semantics/objects
```

К следующей неделе переходите после визуальной проверки собственных detections,
проверки проекции на нескольких объектах и наблюдения штатных отказов при
задержке/неоднозначности. Не компенсируйте ошибки калибровки ослаблением порогов.

В этой среде проверены ROS-free геометрия, отсев фоновой стены,
неоднозначные кластеры, timestamps, калибровка, TTL/ID и вызов API с mock.
Дополнительно запущен настоящий pretrained detector с закреплёнными весами на
образце Hugging Face/COCO 640×480: найдены две кошки и два пульта, CPU-инференс
занял 9.17 с. Отчёт: `validation/grounding_dino_image_smoke.json`. Это проверка
загрузки весов и обработки изображения, а не качество на вашей комнате или
робототехнический benchmark. **9 с на CPU несовместимы с пределом live-кадра
0.5 с**: такая live-локализация будет отклонена. Проверяйте задержку на GPU,
не увеличивайте допустимый возраст, чтобы скрыть медленный инференс.
ROS runtime/TF, live fusion и качество person/chair/table/door здесь не проверены.
Успешный benchmark по синтетическому человеку из геометрических фигур не заявляется.
