# Проверка поставки v2

**Финальный общий прогон: 353 теста прошли, ошибок и пропусков нет.** Дополнительно выполнены синтетический data → train → evaluation → closed-loop pipeline, проверка обнаружения двух ROS-пакетов, синтаксиса Python/shell и структуры поставки. На реальном роботе эта поставка не запускалась.

## Общий прогон

```bash
python -m pytest -q --junitxml=validation/pytest.xml
```

Фактический вывод: `353 passed in 4.63s`. JUnit сообщает 353 cases, 0 failures, 0 errors, 0 skipped, время 4.627 s. Это время тестов на подготовительной машине, не частота управления. Parametrized cases учитываются как отдельные тесты.

Машиночитаемые результаты: [pytest.xml](../validation/pytest.xml), [summary.json](../validation/summary.json), [environment.json](../validation/environment.json), [release_checks.json](../validation/release_checks.json).

| Проверяемая область | Что именно проверено программно |
|---|---|
| Камера и часы | SOE/EOF/EMPIRICAL intervals, clock/flag ошибки, будущие/reversed timestamps, unknown timing, фактический Linux V4L2 ABI через host C compiler, latest-only capture/decode |
| Геометрия и guard | Старые кадры, replay/epoch, lease, deadman, разрыв таймера, возраст от earliest exposure, полная экспозиция после settling, наблюдаемость swept footprint, тормозной запас, запрет обратного хода/слепых поворотов |
| Подтверждение остановки | Поздний MCU PWM-off увеличивает settling; старый ACK не сокращает его; будущий stop timestamp отзывает разрешение |
| Serial и прошивка | CRC, boot/nonce/seq, incomplete/duplicate/missing ACK, фактические tick durations, reboot, tick wrap, interrupted pulses, независимое прекращение pulse, UART flooding и EEPROM fault cases |
| Dispatcher и ввод привода | Один send на pulse_id, немедленный stop при ошибке, ограниченное ожидание явного ARM, commissioning dry run и mocked одиночный импульс |
| Typed ROS bridge | Кадр и JPEG одной identity, полная MCU история, round-trip metadata, границы размеров; fixtures повторяют формы generated messages |
| ROS actions и CLI | Новая задача не наследует ARM/lease; stop/cancel до execute; ACK принадлежит своему goal и считается один раз; гонка success/disarm; закрытие во время inference; STOP после discovery; STOP до ожидания cancel |
| Raw → dataset | MCU-confirmed coverage, schema v2, время экспозиции, непрерывность истории, границы/faults, точные сегменты, immutable metadata, отсутствие подставленных нулей |
| Обучение и оценка | Формы/loss/masks, причинная работа filter, checkpoint provenance, неделимые сессии/layout/day, hold-out без копий, common-mask ablation, PWM-квантизация и MPC |
| Допуск | Synthetic и несовместимое происхождение не проходят live loader; trial/full evidence проверяют привязку к checkpoint и calibration; незаполненные примеры отклоняются |

### Что означает проверка прошивки

`tests/test_firmware.py` компилирует настоящий `roomwm_drive.ino` с подставленным Arduino API через host `g++`, `-Wall -Wextra -Werror` и UndefinedBehaviorSanitizer. Исполняется логика скетча, включая вызовы общего `timerTick()` и проверку simulated UART/EEPROM failures.

Это **не AVR target build**: реальный ISR, регистры таймеров, электрические сигналы, загрузчик, точность тактового генератора и работа платы здесь не измеряются. Время 100 MCU ms не следует объявлять измеренными 100 SI ms по результатам host теста.

### Что означает проверка ROS

Тесты `test_ros_actions.py` извлекают действительный `PCNode` и вызывают его callbacks с контролируемым порядком событий. `test_goal_client.py` вызывает настоящий `goal_client.main` с fake ROS окружением. Это проверяет логику при гонках, но не работу executor/DDS на конкретной версии rclpy.

Фактическое обнаружение colcon с явными `--paths` вернуло:

```text
room_world_model  (ros.ament_python)
roomwm_msgs       (ros.ament_cmake)
```

Сборка `rosidl_generate_interfaces`, импорт сгенерированных Python types, action discovery, QoS совместимость, DDS fragmentation большого JPEG и сеть Pi↔PC должны быть проверены на ROS 2 Jazzy. В этой среде `rclpy` отсутствовал; реальный ROS build не заявляется. Для него в поставке есть `scripts/build_ros.sh`.

## Синтетический сквозной прогон

Запуск воспроизводится командой:

```bash
python -m room_world_model.smoke --output runs/smoke_v2
```

Нужен новый или пустой каталог. `bash scripts/smoke.sh` запускает тот же pipeline, затем pytest. Runner не открывает camera/serial, не требует ROS graph и не скачивает perception-веса.

Фактически выполнена последовательность:

1. Шесть synthetic episodes/layouts по 24 кадра, seed 100.
2. Train/validation/test с разделением по layout.
3. Две CPU эпохи модели latent 16, hidden 24, ensemble 2; два train batch и один validation batch на эпоху, batch size 2.
4. Held-out action ablation на четырёх окнах: actual, zero, permuted и persistence с одинаковой valid-label mask.
5. Четыре paired toy-сценария, каждый для reactive и WM-MPC, до 64 шагов; выбранные действия изменяют toy scene.
6. Загрузка checkpoint и подтверждённый отказ `for_live=True` для synthetic weights.

Это намеренно малый smoke, который проверяет соединение компонентов. Он не заменяет обучение на записях комнаты.

### Фактический отрицательный результат проверки пользы

| Прогноз | Pixel MSE ↓ | Aux MAE на общей маске ↓ |
|---|---:|---:|
| WM, actual actions | 0.04217310 | 0.29143036 |
| WM, zero actions | 0.04217309 | 0.29132384 |
| WM, shuffled actions | 0.04217310 | 0.29141070 |
| Persistence | 0.00014943 | 0.01153067 |

Модель уступила persistence по обеим приведённым ошибкам. Actual/zero/shuffled почти не различаются: полезная зависимость прогноза от действия этим запуском не доказана. Часть task metrics равна `null`, поскольку в выбранных четырёх окнах нет соответствующих валидных positive/near-obstacle examples; `null` не является нулевой ошибкой.

| Toy controller | Успешные задачи | Столкновения в toy model | Поведение |
|---|---:|---:|---|
| Reactive baseline | 1 / 4 | 0 | Выполнял допустимые движения |
| WM-MPC | 0 / 4 | 0 | Выбирал STOP на всех шагах |

Ноль столкновений у неподвижной модели не означает успеха навигации. Pipeline завершился корректно, **полезность модели для управления не подтверждена**. Ни один результат этого smoke не создаёт real-robot acceptance. Его веса в ZIP не включены.

## Сохранённые отчёты

| Файл | Содержимое |
|---|---|
| [smoke_training.json](../validation/smoke_training.json) | Настройки, разделение, происхождение и фактические training/validation losses |
| [smoke_offline_evaluation.json](../validation/smoke_offline_evaluation.json) | Held-out окна, common masks, ablation, horizon errors и ограничения метрик |
| [smoke_closed_loop.json](../validation/smoke_closed_loop.json) | Начальные условия, действия, траектории и сравнение двух controller |
| [smoke_execution_log.json](../validation/smoke_execution_log.json) | Фактически выполненные команды, return codes и вывод |
| [smoke_summary.json](../validation/smoke_summary.json) | Короткая сводка, `synthetic_live_gate_rejected=true`, `real_robot_acceptance=false` |

Пути в execution log отражают подготовительную среду. Для повторения используйте команду runner выше, а не её старые абсолютные пути. Прежние synthetic reports из v1 удалены из поставки, чтобы их нельзя было принять за результат v2.

## Окружение и оставшиеся проверки

Фактическая среда: Linux x86_64, Python 3.12.14, CPU PyTorch 2.14.1+cpu, NumPy 2.5.4, OpenCV headless 5.0.0.93, Pillow 12.3.0, pytest 9.1.1, PyYAML 6.0.3, pyserial 3.5, g++ 13.3.0. Это запись окружения выполненного прогона, не требование устанавливать те же версии поверх ROS на роботе. Стандартная модель отдельно создана и содержит **1 960 515 параметров**.

До испытаний в комнате остаются реальные ROS build/network/QoS, загрузка perception на GPU и измерение inference latency, V4L2 режим и оптические timing bounds камеры, AVR-сборка и проверка PWM/watchdog на плате, измерение скорости/выбега/торможения, обзор всего footprint, реальные held-out данные и paired baseline/WM trials. Порядок приведён в [README](../README.md), [аппаратной инструкции](HARDWARE_AND_SAFETY_RU.md) и [миграции](MIGRATION_V2_RU.md).
