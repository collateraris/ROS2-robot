# Переход с v1 на v2 и соответствие Deep Research

**v2 сохраняет компактную GRU world model и stop–observe–pulse архитектуру, укрепляя происхождение времени, действий и результатов проверки.** Версия не содержит весов, обученных в вашей комнате, и не подтверждает физическую автономность. Поставляемая конфигурация сохраняет отключённые моторы.

## Что из исследования реализовано

| Требование Deep Research | Реализация в v2 | Граница результата |
|---|---|---|
| Время приобретения кадра вместо времени окончания `VideoCapture.read()` | `camera_v4l2.py`, `timebase.py`, `frames.py`: native DQBUF, V4L2 flags/clock, SOE/EOF/empirical exposure bounds, latest frame | Семантика timestamp и численные границы должны быть проверены на физической камере |
| Измерение оптической задержки и buffering | `timing_calibration.py`, команды `calibrate v4l2-probe`, `latency-record`, `latency-fit` | GPIO LED и отдельные fit/check recordings нужны на роботе; программный smoke их не заменяет |
| Фактически подтверждённые действия MCU | `actuator.py`, `firmware/roomwm_drive/roomwm_drive.ino`: boot/nonce/seq/pulse identity, START/STOP ticks, фактические PWM, stop reason, duration/uncertainty | MCU подтверждает PWM и его время; скорость/выбег колёс требуют отдельных измерений |
| Причинный dataset | `record.py`, `data.py`: pulse и idle отдельными сегментами, полная история, маски неизвестных переходов, bounds экспозиции, schema v2 | Среднее в `actions` служит диагностикой; оно не может восстановить пропущенный импульс |
| Typed ROS 2 control interfaces | `ros2/roomwm_msgs/msg`, `action/ApproachPerson.action`, `ros_interfaces.py`, `ros_pi.py`, `ros_pc.py`, `goal_client.py` | Нужна ROS-сборка interfaces; DDS между реальными Pi/PC проверяется при вводе в эксплуатацию |
| Разделение perception, learned prediction и независимого gate | `perception.py`, `engine.py`, `planner.py`, `safety.py`, Pi execution | Предсказание свободного места не отменяет ограничения текущей геометрии и UNKNOWN |
| Калибровка метрической глубины | `calibrate depth`, `CameraCalibration`, `label.py`, calibration/label fingerprints | Pseudo-labels остаются зависимыми от учителя; реальные расстояния и ошибки требуют измерений |
| Whole-session/layout/day hold-out | `data.partition_episodes`, `train.py`: train/validation/test, session union, selected hold-out axis, content fingerprints, `split_audit` | Новый layout не означает автоматически новый день/свет; отчёт показывает пересечения |
| Action-conditioned multi-step training | `model.py`, `train.py`: точные ordered segments, teacher-forced и open-loop losses, masked auxiliary targets, ensemble bootstrap | Это компактная deterministic GRU ensemble; полный stochastic RSSM не заявлен |
| Проверяемая чувствительность к действию | `evaluate.py`: actual/zero/shuffled action и persistence на общей valid-label mask | Результат может оказаться отрицательным; это повод улучшать модель/данные |
| Реализуемые MPC primitives | `engine.py`, `planner.py`: PWM-квантизация до оценки моделью; один набор дискретных действий на горизонте | Исполняется только первый pulse; нет автоматического выполнения всего плана |
| Сравнение с reactive controller | `baseline_controller.py`, `closed_loop.py`, real evidence в `acceptance.py` | Toy benchmark проверяет программный цикл; реальная сравнительная полезность требует paired physical trials |
| Постепенный ввод WM в управление | Shadow → reviewed real ablation → явный `wm_trial` → reviewed paired real trials → обычный WM | Ни обучение, ни synthetic success не создают допуск и не включают ARM |

Подробности модели и метрик: [WORLD_MODEL_RU.md](WORLD_MODEL_RU.md). Сборка и основной запуск: [README](../README.md). Камера, привод и калибровки: [HARDWARE_AND_SAFETY_RU.md](HARDWARE_AND_SAFETY_RU.md) и [DATA_AND_CALIBRATION_RU.md](DATA_AND_CALIBRATION_RU.md).

## Несовместимости v1, которые нельзя скрыть переименованием

1. **Firmware/serial protocol.** V1 acknowledgement по времени доставки на Pi не подтверждает реальный MCU interval. Для v2 нужны совместимые firmware и Python transport, matched START/STOP и новые identity/uncertainty fields. Перепрошивка и проверка железа выполняются отдельно; архив не подключается к вашему Arduino.
2. **Camera envelopes.** V1 `read` timestamps не превращаются автоматически в приобретение изображения. V2 требует явных валидных exposure bounds. Некалиброванный read-time fallback пригоден для диагностики/shadow, но не для подтверждённого движения или training transition.
3. **ROS messages.** V1 critical JSON/`String` publishers несовместимы с v2 typed topics. Пересоберите `roomwm_msgs`, установите обновлённый Python package и запустите v2 узлы на обеих машинах. Нельзя смешивать старый Pi и новый PC.
4. **NPZ.** Loader требует `schema_version=2`, точных `action_segments`, `segment_mask`, новой action semantics и MCU provenance. Для реальных данных обязательны integer exposure bounds. Переименование старых metadata не создаёт этих фактов.
5. **Checkpoint.** Формат теперь `room-world-model-v2`. V1 weights отклоняются: их следует переобучить на допустимых v2 данных. Совпадения размерностей сети недостаточно для совпадения action/timing semantics.
6. **Config/calibration fingerprints.** Используйте `config/robot.yaml` v2 как структуру, перенося только проверенные значения. Новые timestamp, depth и action settings меняют fingerprint; старые reviewed model evidence не относятся автоматически к новой конфигурации.

Для старой NPZ есть исключительно read-only проверка:

```bash
python -m room_world_model.data --audit-v1 data/legacy_episode.npz
```

Повторный export допустим только если исходные raw logs уже содержат достаточные MCU и camera факты. Если их нет, сохраните v1 отдельно для старых offline экспериментов и соберите новые v2 записи. Опции незаметного fallback на interval mean больше нет.

## Точный порядок ввода в работу

### 1. Сборка, калибровки и shadow

Соберите typed interfaces и пакет по README; проверьте согласованность версий Pi/PC/MCU. Измерьте optical latency, speed/braking/settle, camera/depth geometry и UART timing. Нельзя считать числа из example config измерениями вашего робота. Первые проверки выполняются в shadow; реальная baseline/teleop запись требует остальных явно пройденных hardware gates.

Для уже полученного совместимого checkpoint наблюдение за WM без движения:

```bash
roomwm_pc --config /absolute/path/config/robot.yaml \
  --mode shadow --shadow-controller wm \
  --checkpoint /absolute/path/runs/room_v2/best.pt
```

### 2. Реальные данные, offline ablation и рассмотрение результатов

Соберите и разметьте v2 демонстрации при разных расстановках/днях/свете. Одна сессия неделима; default training требует минимум три независимых layout-группы:

```bash
python -m room_world_model.train --data data/labelled --output runs/room_v2 \
  --split-by layout --epochs 30 --sequence-length 8 --burn-in 2 --device cuda
python -m room_world_model.evaluate --data data/labelled \
  --checkpoint runs/room_v2/best.pt --split test \
  --output runs/room_v2/test_ablation.json --device cuda
```

В этой команде `sequence-length=8` означает восемь переходов и девять кадров. 30 эпох не являются достаточным числом по определению. Test не используется для подбора `best.pt`. Если фактические действия не выигрывают у zero/shuffled/persistence, модель остаётся в shadow.

После анализа **реальных** held-out данных заполните отдельную копию [wm_trial_eligibility.example.json](../examples/wm_trial_eligibility.example.json). Нужны точный SHA256 checkpoint, текущий calibration fingerprint, идентификатор исходного отчёта, общий valid-label mask, измеренные ошибки, reviewer и время рассмотрения. Пустой пример, `null`, synthetic statistics и сам факт завершившегося training не проходят `load_trial_eligibility`.

### 3. Ограниченные supervised WM trials

После настоящего рассмотрения evidence настройте `runtime.world_model_trial_file`, `runtime.world_model_sha256` и отдельный `runtime.allow_wm_trials`. Запрос режима должен быть явным:

```bash
roomwm_pc --config /absolute/path/config/robot.yaml --mode wm --wm-trial \
  --checkpoint /absolute/path/runs/room_v2/best.pt
```

`--wm-trial` действует только вместе с `--mode wm`. Pi распознаёт controller `wm_trial`, проверяет именно trial eligibility и не смешивает его с обычным WM допуском. ARM, удерживаемый deadman, текущая геометрия, свежесть кадра, pulse cap и STOP/watchdog действуют и здесь. Этот режим позволяет собрать реальные сравнения до появления окончательного допуска и устраняет замкнутый круг «сначала prove paired trials, но trials полностью запрещены».

### 4. Paired реальные испытания и нормальный WM

Сравните reactive и WM с одинаковыми layout, start pose, target, speed limits, perception и safety gate; фиксируйте успехи, коллизии, минимальные измеренные расстояния, вмешательства и время. Время сравнивается только на парах, где **оба** controller завершили задачу. Реальный trial не заменяется toy trajectory.

Заполните копию [wm_acceptance.example.json](../examples/wm_acceptance.example.json). `load_acceptance` повторно проверяет bound real ablation и требует paired physical evidence: ноль WM collisions, неухудшение intervention count/minimum clearance/success count и улучшение successes либо времени общих успешных пар. Loader проверяет согласованность представленной записи, но не удостоверяет правдивость вручную внесённых чисел.

После рассмотрения этого файла обычный WM использует `runtime.world_model_acceptance_file`, `runtime.world_model_sha256`, `runtime.world_model_validated` и `--mode wm` **без** `--wm-trial`. Изменение checkpoint или связанной калибровки требует нового соответствующего evidence. Включение любого флага в YAML само по себе не заменяет остальные проверки Pi и оператора.

## Что проверено в поставленном архиве и что ещё предстоит

Воспроизводимый программный smoke запускается одной командой, не обращается к camera/serial/ROS и не включает реальные моторы:

```bash
python -m room_world_model.smoke --output runs/smoke_v2
```

Фактические отчёты лежат в `validation/smoke_*.json`. Маленькая двухэпоховая synthetic модель уступила persistence, не показала полезного action conditioning и завершила **0/4** toy tasks против **1/4** у reactive. Оба controller получили ноль toy collisions; WM при этом выбирал STOP. Это свидетельство исполнимого программного контура и отрицательного результата проверки полезности, а не обученной автономности. Синтетические weights в архив не включены.

Для вашей комнаты остаются сбор реальных данных, аппаратные калибровки, проверка ROS сети и физические paired trials. Нет опубликованной в проекте скорости perception/MPC на вашей RTX 4090. Нет глобального SLAM, гарантированного выхода из тупиков, полного stochastic RSSM, learned odometry из энкодеров или гарантии поведения при произвольных быстро движущихся людях. Передняя камера имеет невидимые зоны; UNKNOWN не становится подтверждённым FREE из-за уверенности world model. При недостаточных наблюдениях или отсутствии доказанной выгоды правильный рабочий режим — остановка/shadow и дальнейшее улучшение данных.
