# USB-камера: драйвер, калибровка, исправление изображения и TF

Это пример для обычной **USB UVC/V4L2-камеры на Ubuntu 24.04 / ROS 2 Jazzy**.
Модель вашей камеры неизвестна: устройство, разрешение и формат нужно проверить.
Для CSI- или сетевой камеры требуется её драйвер, но выходные ROS-топики должны
соответствовать той же схеме. В Gazebo камера и CameraInfo уже публикуются:
этот USB-драйвер и дополнительные TF в симуляции **не запускайте**.

Итог: `/camera/image_raw` → `image_proc` → `/camera/image_rect`, рядом
`/camera/camera_info`; frame изображений — `camera_optical_frame`.
Intrinsics описывают объектив, extrinsics — положение камеры относительно LiDAR.
Одно не заменяет другое. Начальные проверки выполняйте с отключёнными моторами.

## 1. Установка и выбор устройства

Из корня распакованного проекта:

```bash
source scripts/env_robot.sh
sudo apt update
sudo apt install -y ros-jazzy-usb-cam ros-jazzy-camera-calibration \
  ros-jazzy-image-proc v4l-utils
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl --device /dev/video0 --list-formats-ext
```

Замените `/dev/video0` найденным устройством. У одной камеры могут быть несколько
video-узлов; нужен узел захвата изображения, а не metadata. Для постоянной
настройки предпочтителен уникальный путь `/dev/v4l/by-id/...-video-index0`.
При отказе доступа проверьте права устройства и членство в группе `video`;
после `sudo usermod -aG video "$USER"` полностью выйдите и войдите в сеанс.

Создайте конфигурацию драйвера. Это **пример режима захвата**, не intrinsics:
оставьте 640×480, 30 Гц и `mjpeg2rgb` только если камера поддерживает MJPG в этом
режиме. `pixel_format` — имя преобразования **usb_cam**, не буквальный V4L2 FourCC.

```bash
mkdir -p config/camera
cat > config/camera/usb_cam.yaml <<'YAML'
/**:
  ros__parameters:
    video_device: /dev/video0
    image_width: 640
    image_height: 480
    framerate: 30.0
    pixel_format: mjpeg2rgb
    io_method: mmap
    camera_name: room_camera
    frame_id: camera_optical_frame
    camera_info_url: ""
YAML
```

Отредактируйте устройство и режим по выводу V4L2. Имена поддержанных
преобразований указаны в [официальном usb_cam](https://github.com/ros-drivers/usb_cam#driver-supported-formats).
Не подставляйте чужие `K`, `D` и `P` из примеров или симулятора.

## 2. Запуск камеры

В каждом новом терминале из корня проекта выполняйте `source scripts/env_robot.sh`.
Терминал 1:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --params-file "$ROOM_PROJECT_ROOT/config/camera/usb_cam.yaml" \
  -r __ns:=/camera -r __node:=room_camera
```

До калибровки предупреждение об отсутствующем calibration YAML ожидаемо; нулевая
матрица CameraInfo ещё не разрешает метрическую привязку. Терминал 2: каждую
проверку частоты остановите через Ctrl+C перед следующей командой.

```bash
source scripts/env_robot.sh
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

Посмотрите `/camera/image_raw` в дисплее Image программы RViz. Проверьте цвет,
размеры, отсутствие зеркального переворота и frame `camera_optical_frame`.
Должны поступать и Image, и CameraInfo с согласованными header stamps.
Не запускайте второй драйвер на той же камере.

## 3. Измерить intrinsics шахматной доской

Возьмите ровную шахматную доску. `--size` означает **число внутренних углов**,
`--square` — измеренную сторону клетки в метрах. Например, для доски с 8×6
внутренними углами и клеткой действительно 25 мм:

```bash
source scripts/env_robot.sh
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --camera_name room_camera \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

Замените параметры своей доской. Перемещайте её к краям и центру кадра, меняйте
расстояние и наклон; во время выборки держите неподвижно. Затем нажмите
**CALIBRATE**, оцените исправленное изображение и нажмите **SAVE**. В Jazzy SAVE
пишет `/tmp/calibrationdata.tar.gz`, внутри моно-калибровки находится `ost.yaml`:

```bash
tar -tzf /tmp/calibrationdata.tar.gz
tar -xOf /tmp/calibrationdata.tar.gz ost.yaml > config/camera/intrinsics.yaml
```

Это измеренный файл, содержащий размеры изображения, distortion model и матрицы
`camera_matrix`, `distortion_coefficients`, `rectification_matrix`,
`projection_matrix`. Проверьте имя камеры и разрешение. COMMIT использует сервис
`/camera/set_camera_info`; для этого руководства достаточно SAVE и явной загрузки
файла. Если calibrator не видит сервис, проверяйте namespace; для сохранения
через SAVE допускается запуск с `--no-service-check`, без обещания COMMIT.

Остановите только usb_cam через Ctrl+C и запустите его с измеренным файлом:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --params-file "$ROOM_PROJECT_ROOT/config/camera/usb_cam.yaml" \
  -r __ns:=/camera -r __node:=room_camera \
  -p camera_info_url:="file://$ROOM_PROJECT_ROOT/config/camera/intrinsics.yaml"
```

После смены разрешения, фокуса, zoom или crop калибровку нужно проверить заново.
Зафиксируйте фокус для работы, если камера позволяет; intrinsics от другого
положения фокуса могут давать ошибку.

## 4. Получить исправленное изображение

Загрузка intrinsics в драйвер **не исправляет сами пиксели**. В отдельном
терминале запустите проверенный по исходникам Jazzy executable `rectify_node`:

```bash
source scripts/env_robot.sh
ros2 run image_proc rectify_node --ros-args \
  -r __ns:=/camera \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

Проверьте `ros2 topic hz /camera/image_rect` и покажите этот топик в RViz Image.
Узел может ждать подписчика перед обработкой. Прямые края должны выглядеть
прямыми; пустой/сильно искажённый кадр — повод проверить calibration и режим
захвата. Сохраняйте соответствие `/camera/image_rect` ↔ CameraInfo с его `P`:
не масштабируйте/обрезайте изображение отдельно от CameraInfo. Для этой
моно-камеры используется та же `/camera/camera_info`; драйвер публикует измеренные
`K/D/R/P`, а rectifier применяет их к изображению.

## 5. Добавить измеренные TF камеры

У `base_link` оси X вперёд, Y влево, Z вверх. Определите `camera_link` в
оптическом центре камеры с X по направлению взгляда, Y влево, Z вверх.
Измерьте его позицию и ориентацию относительно `base_link`. В следующем шаблоне
**все `MEASURE_...` нужно заменить числами**; метры и радианы:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x MEASURE_X_M --y MEASURE_Y_M --z MEASURE_Z_M \
  --roll MEASURE_ROLL_RAD --pitch MEASURE_PITCH_RAD --yaw MEASURE_YAW_RAD \
  --frame-id base_link --child-frame-id camera_link
```

Optical frame по соглашению имеет X вправо, Y вниз, Z вперёд. Для определённого
выше `camera_link` преобразование осей фиксировано; другой терминал:

```bash
source scripts/env_robot.sh
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll -1.57079632679 --pitch 0 --yaw -1.57079632679 \
  --frame-id camera_link --child-frame-id camera_optical_frame
```

Это два постоянно работающих процесса. После проверки перенесите эти же
измеренные transforms в собственный launch/URDF. Если их уже публикует
`robot_state_publisher` или драйвер камеры, второй источник не запускайте.
`base_link → laser` уже создаёт аппаратный bringup; не добавляйте второй parent
для `laser` или `camera_optical_frame` и не публикуйте отдельно дублирующее ребро
`laser → camera_optical_frame`.

При работающей навигации проверьте каждую команду отдельно; `tf2_echo` работает
до Ctrl+C:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo camera_optical_frame laser
```

Второй вызов показывает transform, переводящий точку `laser` в
`camera_optical_frame`: именно его translation и Quaternion XYZW нужны в
`camera_from_lidar` файла `config/week02/calibration.yaml` из WEEK02.
Они должны следовать из измеренной геометрии; согласование двух копий
выдуманных чисел физическую калибровку не выполняет.

## 6. Подключить к восприятию недели 2

Скопируйте шаблоны по WEEK02 и задайте в `config/week02/perception.yaml`:

```yaml
room_perception:
  ros__parameters:
    image_topic: /camera/image_rect
    camera_info_topic: /camera/camera_info
    camera_frame: camera_optical_frame
    image_is_rectified: true
```

Это фрагмент для изменения скопированного файла; остальные настройки сохраните.
В `calibration.yaml` заполните измеренный `camera_from_lidar`. После проверки
геометрии, исправленного кадра и соответствия TF задайте там `verified: true`
и `image_is_rectified: true`. Формат проверяется командой:

```bash
python -m room_semantics.validate_calibration config/week02/calibration.yaml --require-verified
```

Затем используйте аппаратную launch-команду perception из WEEK02. Проверьте
на нескольких неподвижных объектах, расстояниях и участках кадра: реальный
объект, bbox и метрическая лазерная опора должны соответствовать друг другу.
Измерьте расстояние независимо. Сначала работайте стоя, затем проверяйте малые
скорости и задержки. Отсутствие metric objects допустимо: плоский LiDAR может не
пересекать нужную часть объекта. В поставке нет автоматического инструмента
измерения camera–LiDAR extrinsics.

Header stamp драйвера сам по себе не доказывает отсутствие буферизации камеры.
Измеряйте задержку всей цепочки и не заменяйте время старого кадра на «сейчас».
При разнесении ПК и робота сначала обеспечьте согласованные часы.

## Проверенные первичные источники

- [usb_cam: запуск и параметры ROS 2](https://github.com/ros-drivers/usb_cam),
  [исходный пример YAML](https://github.com/ros-drivers/usb_cam/blob/ros2/config/params_1.yaml).
- [camera_calibration: моно-калибровка, ветка Jazzy](https://github.com/ros-perception/image_pipeline/blob/jazzy/camera_calibration/doc/tutorial_mono.rst),
  [опции cameracalibrator](https://github.com/ros-perception/image_pipeline/blob/jazzy/camera_calibration/src/camera_calibration/nodes/cameracalibrator.py).
- [image_proc Jazzy: rectify_node](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/CMakeLists.txt),
  [вход image и выход image_rect](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp).
- [REP 103: оси optical frame](https://www.ros.org/reps/rep-0103.html),
  [tf2 static broadcaster](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html).

Команды сверены с API/исходниками; физическая USB-камера, GUI калибровки и
ROS/Gazebo runtime в среде подготовки не запускались.
