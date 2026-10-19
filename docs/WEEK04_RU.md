# Неделя 4: воспроизводимая оценка

Цель — отличить успешное завершение ROS action от действительно выполненной задачи.
Пакет генерирует комнаты Gazebo, записывает события отдельных эпизодов и строит
CSV, JSON и график по настоящим логам. Включённый `smoke_report` — **только
синтетическая проверка вычисления метрик**, это не поездки Gazebo и не результаты
YOLO/LLM. Ни Gazebo, ни реальные видео в этой среде не запускались.

## Что считается экспериментом

| Сравнение | Общие условия | Что оно показывает |
|---|---|---|
| `rules` против `llm` | Те же команда, сцена, начальная pose, perception, сохранённые веса и настройки Nav2 | Вклад символьного планировщика при одинаковых доступных наблюдениях |
| `oracle_goal` | То же шасси, LiDAR, RF2O, Nav2; цель дана из evaluator GT | Диагностика навигации с правильной целью; семантика полностью обойдена |
| Синтетический fixture smoke | Заранее записанные искусственные координаты и исходы | Только правильность кода метрик |

`oracle_goal` имеет привилегированный доступ к цели и **не является равноправным
конкурентом всей системе «камера → смысл → движение»**. Его результат отвечает на
вопрос: смогла бы навигация доехать, если правильная цель уже известна?

Генератор рисует цилиндр вместо человека и простые блоки вместо мебели. Это
**геометрические proxies**, а не датасет фотореалистичных людей. Предобученный
детектор может их не распознать. Результат такого отказа следует записать как
отказ perception, а не добавлять скрытую позицию человека в `/room_semantics/objects`.
GT manifest читает только evaluator. Из GT нет издателя semantic graph.

Для запуска без настоящего распознавания есть отдельные задачи с заранее известными
именованными map-точками `table` и `home`. Команда fixture `поезжай к table` использует
именно известную точку; она не проверяет понимание слова «стол». Для `домой` имеется
явное правило. Команда `подъедь к человеку` требует действительно наблюдаемый объект.

## Генерация сцен

Из корня проекта:

```bash
source scripts/env_robot.sh
python -m room_evaluation.scenes --split all --count 3 --out weeks/week04/generated
```

Получатся `train_000..002`, `dev_000..002`, `test_000..002` с файлами:

- `room.sdf`: законченный world `room`, стены, obstacles и `model://room_robot`;
- `manifest.json`: seed, размеры комнаты, spawn, evaluator GT, команды и критерии;
- `known_waypoints.yaml`: только явно известные `table`/`home` в начальной системе `map`.

Seeds трёх split не пересекаются: `100+i`, `10000+i`, `20000+i`. Диапазоны ширины
и высоты также различны: train 5.5–6.5 × 4.5–5.5 м, dev 7–7.5 × 5.5–6 м,
test 8–9 × 6.5–7.5 м. Это одновременно смена seed и масштаба комнаты; эти два
влияния в таком маленьком fixture не разделены. Не подбирайте параметры на test.
`--count` должен быть от 1 до 1000. Генерация при одинаковых split/index детерминирована.

Координаты GT — абсолютные координаты Gazebo, `/sim/odom` имеет frame `sim_world`.
RF2O начинает `odom` в начальной pose робота; при новом mapping map выравнивается
по начальной pose. Поэтому известные точки и oracle-цель вычисляются как
`R(-spawn.yaw) * (goal_world - spawn_position)`, yaw уменьшается на spawn.yaw.
Это допущение относится к **новой карте этой сцены**. Для загрузки чужой карты или
ручного изменения initial pose нужно отдельно задать соответствие систем координат;
этот runner для такой схемы не предназначен.

## Один настоящий эпизод Gazebo

Сначала выполните установку и сборку из недели 1. Затем в каждом терминале:

```bash
cd ~/robot/room_robot_v5_four_weeks
source scripts/env_robot.sh
```

Если проект распакован в другой каталог, замените путь. Скрипт также активирует
`.venv`: она нужна для Python-зависимостей отчёта, в том числе matplotlib.

Терминал 1 — отдельный свежий мир для каждого метода/повтора:

```bash
ros2 launch room_sim sim.launch.py \
  world:="$PWD/weeks/week04/generated/dev_000/room.sdf" \
  simulation_autonomy:=true start_mission:=false
```

Терминал 2 — perception недели 2 (команда в `WEEK02_RU.md`). Одинаковая версия
perception и одинаковые веса обязательны для rules и LLM. Именованной точке не
нужен выдуманный semantic object: scene graph может оставаться пустым.

Терминал 3 — rules:

```bash
ros2 launch room_tasks tasks.launch.py \
  backend:=rules use_sim_time:=true shadow_only:=false enable_execution:=true \
  robot_radius_m:=0.19 standoff_m:=1.0 \
  waypoints_confirmed:=true \
  waypoints_file:="$PWD/weeks/week04/generated/dev_000/known_waypoints.yaml"
```

Для LLM в новом запуске сцены замените `backend:=rules` на `backend:=ollama`;
сам Ollama и выбранная модель должны быть запущены заранее по инструкции недели 3.
Одинаковые image/use_image настройки обязательны. По умолчанию планирование работает
по grounded scene graph без картинки; дополнительные изображения меняют условия сравнения.

Терминал 4 — один эпизод:

```bash
ros2 run room_eval evaluate \
  --manifest "$PWD/weeks/week04/generated/dev_000/manifest.json" \
  --task go_table --method rules \
  --output "$PWD/weeks/week04/logs/dev_000_rules.jsonl" \
  --enable-sim-actuation --robot-radius 0.19
```

Для LLM используйте `--method llm`; runner проверит, что backend действительно
`ollama`. Для oracle Nav2 baseline можно не запускать `room_tasks`:

```bash
ros2 run room_eval evaluate \
  --manifest "$PWD/weeks/week04/generated/dev_000/manifest.json" \
  --task go_table --method oracle_goal \
  --output "$PWD/weeks/week04/logs/dev_000_oracle.jsonl" \
  --enable-sim-actuation --robot-radius 0.19
```

Не запускайте hardware bridge вместе с Gazebo. Runner до ARM проверяет:
`use_sim_time=true`, живой `/room_sim/is_simulation`,
`/room_sim/output_mode=simulation`, **единственного** поставщика `/room_base/arm`
с именем `/room_sim_drive`, его `use_sim_time=true` и `simulation_autonomy=true`.
Без явного `--enable-sim-actuation` команда прекращается до ARM. Автоматического
армирования реального робота здесь нет. В симуляции ARM отправляется только после
построения плана/пути и события принятой навигации; ожидание LLM проходит с
отключёнными моторами и не расходует watchdog grace period. Выполняется не более
одного запроса ARM на эпизод; повторного ARM после остановки нет.

Сначала получите стабильные `/scan`, RF2O `/odom`, `/room_base/odom_healthy=true`,
активный Nav2 и карту. Если ARM отклонён, устраните причину и начните новый эпизод.
Runner не снимает estop и не восстанавливает поездку автоматически.
`--task all` выполняет задачи последовательно в **одном** мире; последующая задача
начинается там, где завершилась предыдущая. Для честного парного сравнения
используйте один `--task` и перезапускайте мир между trials.
Перезапуск Gazebo выполняется оператором; runner не выдаёт незаявленный reset.

## Измеряемые метрики

| Метрика | Источник и правило |
|---|---|
| Success | Завершённая задача + свежая GT final pose внутри допуска + нужный yaw + без obstacle contacts и вмешательств |
| Человек | `abs(distance(robot, person) - 1.0 м) <= 0.25 м`, heading в сторону человека с ошибкой ≤0.4 рад |
| Известная точка | Расстояние до заданной pose ≤0.25 м, yaw error ≤0.4 рад |
| Время | Отдельно `/clock` simulation seconds и monotonic wall seconds; wall включает замедление, ожидание LLM и rendering |
| Path length | Сумма перемещений настоящей GT `/sim/odom`; RF2O/desired cmd не используются |
| Collisions | Только реальные `ros_gz_interfaces/Contacts` из `/sim/contacts`; считаются уникальные robot-obstacle пары, ground/self исключаются; contact bumper имеет ограниченную высоту |
| Clearance | Минимальный конечный LiDAR ray минус `--robot-radius`, минимум 0; приближённая геометрическая метрика, не факт столкновения |
| Вмешательства | Число событий String `/room_tasks/intervention`; физическое вмешательство оператор отмечает этим сообщением |
| Причина отказа | Task failure/reject, GT цель не достигнута, collision, intervention, timeout, abort либо отсутствующие данные |

Пример регистрации вмешательства:

```bash
ros2 topic pub --once /room_tasks/intervention std_msgs/msg/String '{data: "operator canceled unsafe approach"}'
```

Это **не** автоматический детектор человеческих касаний. Потеря deadman/estop
приведёт к остановке; её причину дополнительно видно в ROS status.
Контактный bumper модели покрывает горизонтальную проекцию корпуса и колёс, но
не гарантирует обнаружение препятствия целиком ниже его высоты. Это ограничивает
контактную метрику данного fixture; нельзя объявлять её физически полной.
Gazebo ContactSensor обычно публикует только непустые контакты. В модели робота
дополнительно включены датчики контактов колёс с полом в том же `/sim/contacts`: они
служат живым свидетельством работы контактного канала. Эти реальные floor samples
учитываются для coverage, но **не** считаются obstacle collisions; ongoing obstacle
contact одной и той же пары также не умножает счётчик на частоту sensor updates.
Если bridge вообще не прислал Contacts, отсутствие сообщений нельзя считать доказательством
отсутствия столкновений: отчёт пометит trial `incomplete`. Потеря coverage контактов
на >1 с или LiDAR на >0.6 с также делает отчёт неполным. Пропуски/скачки GT,
устаревшая final pose и незакрытый JSONL не дают success.

Если симуляция остановлена/пропал simulation gate/потребовалась ручная отмена,
получится `aborted`. Штатный simulation timeout при полных данных — `failed`.
Сбой данных — `incomplete`. Доля успеха считается только по complete trials,
а числа aborted/incomplete **всегда** показаны рядом; не скрывайте их из отчёта.

## Отчёт и проверка кода

```bash
bash scripts/evaluate.sh \
  --logs weeks/week04/logs/*.jsonl \
  --out weeks/week04/report
```

Создаются `episodes.csv`, `summary.json`, `evaluation.png`. Источники и split
группируются отдельно. Если matplotlib не установлен, установите зависимости
проекта; изображения строятся обычным matplotlib без генерации картинок.

Искусственный smoke без ROS, Gazebo и модели:

```bash
python3 -m room_evaluation.smoke --out weeks/week04/synthetic_fixture.jsonl
bash scripts/evaluate.sh --logs weeks/week04/synthetic_fixture.jsonl --out weeks/week04/smoke_report
python3 -m pytest -q tests/test_evaluation_metrics.py tests/test_evaluation_scenes.py
```

Не цитируйте проценты smoke как качество rules/LLM. Его числа специально
сконструированы, чтобы проверить случай ложного `succeeded` при неверной GT pose.

## План четырёх недель и принимаемые результаты

| Неделя | Проверяемый результат | Что приложить |
|---|---|---|
| 1 | Gazebo → реальные scans → RF2O → guarded odom → SLAM/Nav2 | rosbag/логи TF и свежести, карта, контролируемая остановка при пропаже scans |
| 2 | Image detections + калиброванное LiDAR grounding без GT inputs | Образцы настоящих image/detection/grounded-object сообщений, rejected cases, настройки camera/LiDAR |
| 3 | Rules и локальный LLM выдают ограниченный валидируемый plan; STOP отдельно | Логи планов и отказов, timeouts, shadow-run до включения execution |
| 4 | Независимый GT отчёт, trials на held-out scenes | Manifests/seeds, raw JSONL, configs/веса, CSV/PNG, failure reasons и отдельные incomplete trials |

Минимальный осмысленный прогон после dev-настройки: 3 test scenes × 2 задачи
(известная точка и perception-dependent подход) × 3 метода × 3 повтора = 54 trials.
Это план эксперимента, **не выполненный результат**. Чтобы сравнить только планировщик,
зафиксируйте perception и одинаковые данные; для оценки всей системы разрешены реальные
различия в наблюдении по ходу разных траекторий, но вывод тогда относится ко всей системе.
