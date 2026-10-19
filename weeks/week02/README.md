# Неделя 2: камера, GroundingDINO, метрическая привязка

Основная инструкция: [docs/WEEK02_RU.md](../../docs/WEEK02_RU.md).

Из корня проекта установите CPU-окружение для проверки и подключите его:

```bash
bash scripts/setup_python.sh perception cpu
source scripts/env_robot.sh
```

Для заранее установленной в `.venv` CUDA-сборки используйте
`bash scripts/setup_python.sh perception existing`. Пересборку ROS выполняйте
в отдельном терминале без venv/Conda, до `source scripts/env_robot.sh`.

- `requirements-perception.txt`: опциональные ML-зависимости, Transformers 4.52.3.
- `run_image_smoke.sh`: настоящий детектор на вашем изображении; при первом запуске скачивает веса.
- `perception.sim.yaml`: параметры для pinhole-камеры симулятора week01.
- `calibration.sim.yaml`: только известные extrinsics симулятора; не для реального робота.

Реальные настройки и шаблон калибровки находятся в
`ros2_ws/src/room_perception/config/`. Шаблон `verified:false` намеренно не позволяет
выдать bounding box за метрическую координату объекта. Сначала проверьте
настоящие detections, затем калибровку и проекцию; только после этого week03.

Быстрая проверка математики без ROS и без весов:

```bash
python -m pytest tests/test_semantics_geometry.py tests/test_semantics_detector.py -q
```

Эти тесты проверяют синтетическую геометрию и вызов API с mock, а не качество
детектора. Узлы week02 не публикуют команды движения.
