# Архитектура четырёхнедельного v5

## Контракты

LiDAR C1 или его симуляция → /scan → RF2O → /odom_rf2o → odom_guard → /odom + odom→base_link.
SLAM Toolbox или AMCL даёт map→odom. Симуляторная ground truth /sim/odom доступна **только оценке**, не RF2O/Nav2/планировщику. robot_state_publisher или измеренный static TF задаёт base_link→laser/camera_optical_frame. Владельцы TF не дублируются.

Камера+CameraInfo+калиброванный LiDAR → GroundingDINO boxes → геометрическое сопоставление → /room_semantics/objects.
Положение объекта — оценка лазерного кластера на высоте сканера, а не полный 3D-центр. Подсчёт лучей/кластеров и согласование timestamps ограничивают очевидные ошибки, но не доказывают, что луч принадлежит человеку. Стена за bbox, перекрытия и меняющиеся объекты остаются источниками ошибок. На железе метрическая привязка выключен до подтверждения реальной калибровки.

Команда+наблюдаемые ID объектов+размеченные waypoint → rules или Ollama VLM → строгий список задач.
Разрешённые действия: goto_object, goto_waypoint, wait, stop. LLM не создаёт координаты, не вызывает shell, не публикует cmd_vel. Координаты вычисляются из текущего сенсорного объекта или подтверждённой таблицы точек. Перед движением: свежесть цели, map/grid clearance, Nav2 ComputePathToPose. Затем NavigateToPose. Потеря/движение цели и STOP отменяют задачу.

Nav2 → velocity_smoother → CollisionMonitor → /cmd_vel_safe → аппаратный PWM-мост или симуляционный привод.
Оба привода требуют собственной ARM-сессии и свежих источников. По умолчанию человек удерживает deadman. Только симуляция имеет явно включаемый режим автономного испытания; evaluator дополнительно проверяет provider и маркер симуляции. Не переносить этот режим на реальный serial-мост.

## Разделение пакетов

| Часть | Код |
|---|---|
| Реальное железо без энкодеров | ros2_ws/src/room_base, room_nav, firmware/room_base_rf2o |
| Gazebo и симуляторный привод | ros2_ws/src/room_sim |
| Чистая геометрия/адаптер детектора | room_semantics |
| ROS-восприятие | ros2_ws/src/room_perception |
| Строгие планы/фильтр целей/HTTP backend | room_planning |
| ROS-координатор задач | ros2_ws/src/room_tasks |
| Генерация сценариев/метрики | room_evaluation |
| ROS-оценка только симуляции | ros2_ws/src/room_eval |
| Исследовательские модели v4 | learning_nav |

`start_mission:=false` выключает старый room_mission, когда запущен room_tasks. Не держать несколько независимых отправителей целей Nav2. Новый GUI `python -m robot_extras.task_operator` отправляет /room_tasks/command, старый operator — /room_nav/text_command.

## Что обобщается и как это проверять

Объезд неизвестной геометрии обеспечивает свежий LiDAR+Nav2, а не сам LLM. Open-vocabulary детектор переносит предобученные визуальные и текстовые признаки; это не обещание распознавания любого объекта. VLM выбирает смысл задачи при тех же наблюдениях. Его пользу проверяют сравнением с rules на одинаковом восприятии/сценах; oracle-goal Nav2 — отдельная привилегированная оценка навигации.

Прокси-человек из цилиндров и примитивные стулья проверяют геометрию/интеграцию, но не являются достаточным тестом реального zero-shot распознавания. Fixture detections должны быть явно отмечены, не смешаны с GroundingDINO. Метрики/видео реальных запусков не выдумываются. Веса DINO/VLM скачиваются отдельно, лицензии и версии фиксировать при экспериментах.

## VLA и world model

Для колёсной платформы Nav2 выполняет и конечный подход с отступом. Универсальная манипуляторная VLA не подключается к колёсам как готовый низкоуровневый контроллер.
Включённый TinyVLA — существующий supervised-классификатор известных точек; world model — одношаговый predictor скана/движения. Они сохранены как отдельный offline/shadow эксперимент, не заменяют GroundingDINO+VLM и не обеспечивают learned точное позиционирование. Метаданные schema3 разделяют hardware/gazebo/synthetic; источник принятой команды в sim не называется MCU ACK.

## Сроки и пределы

Четыре недели — порядок проверяемых этапов для прототипа. Это не гарантия готовности всех аппаратных сценариев за 28 дней. Калибровка моторов, камеры, extrinsics, времён и оценка на настоящем роботе могут потребовать дополнительной работы. Автономный подход к движущемуся человеку, манипуляции и универсальный VLA policy не заявлены.

Первичные источники:
- ROS/Gazebo pairing: https://github.com/gazebosim/ros_gz
- Harmonic ROS bridge: https://gazebosim.org/docs/harmonic/ros2_integration/
- GroundingDINO API: https://huggingface.co/docs/transformers/v4.52.3/model_doc/grounding-dino
- Ollama structured outputs: https://docs.ollama.com/capabilities/structured-outputs
- Qwen2.5-VL 7B: https://ollama.com/library/qwen2.5vl:7b
- RF2O/C1 revisions: [../vendor/UPSTREAM.md](../vendor/UPSTREAM.md)
