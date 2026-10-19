#!/usr/bin/env python3
"""Export a self-contained cumulative week snapshot from reviewed shared sources."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
STAGES={'room_semantics':2,'room_planning':3,'room_evaluation':4,
        'room_perception':2,'room_tasks':3,'room_eval':4}
SKIP={'__pycache__','.pytest_cache','.git','.venv','build','install','log','weekly_releases'}


def file_stage(path):
    parts=path.parts
    for key,value in STAGES.items():
        if key in parts:return value
    for value in range(1,5):
        if f'week{value:02d}' in parts or path.name==f'WEEK{value:02d}_RU.md':return value
    if path.name.startswith('test_semantics'):return 2
    if path.name.startswith('test_planning'):return 3
    if path.name=='test_stack_integration.py':return 3
    if path.name=='CAMERA_RU.md':return 2
    if path.name.startswith('test_evaluation'):return 4
    if path.name=='evaluate.sh':return 4
    return 1


def export(week,out):
    available=int((ROOT/'WEEK_STAGE').read_text()) if (ROOT/'WEEK_STAGE').exists() else 4
    if not 1<=week<=available:raise ValueError('requested week is unavailable in this source snapshot')
    paths=[p for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(x in SKIP for x in p.relative_to(ROOT).parts)
           and p.suffix not in {'.pyc','.pyo'} and p.name not in {'SHA256SUMS','WEEK_STAGE'}
           and 'validation' not in p.relative_to(ROOT).parts and file_stage(p.relative_to(ROOT))<=week]
    data={p.relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    data['WEEK_STAGE']=(str(week)+'\n').encode()
    data['README.md']=f'''# Недельный срез {week:02d} из v5

Это самостоятельный накопительный проект: код недель 1..{week}. Распаковывать в отдельную папку, не поверх более позднего среза.

Из корня этой папки, на Ubuntu 24.04 с ROS 2 Jazzy:

```bash
bash scripts/install_system.sh sim
source /opt/ros/jazzy/setup.bash
# Если rosdep ещё не инициализирован: sudo rosdep init
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro jazzy
bash scripts/build_ros.sh all
bash scripts/setup_python.sh {'base' if week==1 else 'all cpu'}
source scripts/env_robot.sh
```

GPU-среда вместо CPU: следовать docs/INSTALL_RU.md, установить подходящий PyTorch и использовать setup_python.sh all existing.
Все пути установки в общей инструкции заменить на фактический корень этого среза.
Начать с docs/WEEK01_RU.md, затем последовательно до docs/WEEK{week:02d}_RU.md. Критерии следующего этапа не пропускать.

Полная исходная поставка v5 содержит больше модулей. Недельный срез проверен на структуру, целостность и доступные импорты отдельно; он не является утверждением об исполненном ROS/Gazebo испытании.
'''.encode()
    data['docs/VALIDATION_RU.md']=f'''# Проверка недельного среза {week:02d}

Срез получен из проверенных исходников полного v5. Архив и SHA256 проверены, исключены будущие ROS-пакеты/чистые модули/тесты. Команда build_ros.sh all выбирает только доступные этапы по WEEK_STAGE. Исполнение ROS/Gazebo и реальное железо требуют отдельной проверки пользователем. Полный протокол тестов находится в основном архиве v5.
'''.encode()
    data['snapshot.json']=json.dumps({'project':'room_robot_v5_four_weeks','through_week':week,'files':len(data)+2,
        'scope':'cumulative source snapshot; no fabricated runtime results'},indent=2).encode()+b'\n'
    manifest=''.join(f'{hashlib.sha256(value).hexdigest()}  {name}\n' for name,value in sorted(data.items()))
    data['SHA256SUMS']=manifest.encode()
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    root=f'room_robot_week{week:02d}'
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for name,value in sorted(data.items()):z.writestr(root+'/'+name,value)
    with zipfile.ZipFile(out) as z:
        assert z.testzip() is None
        for line in manifest.splitlines():
            checksum,name=line.split('  ',1)
            assert hashlib.sha256(z.read(root+'/'+name)).hexdigest()==checksum
    return {'week':week,'files':len(data),'bytes':out.stat().st_size,'archive':str(out)}


def main():
    p=argparse.ArgumentParser(__doc__);p.add_argument('--week',type=int,required=True);p.add_argument('--out',required=True)
    a=p.parse_args();print(json.dumps(export(a.week,a.out),ensure_ascii=False))


if __name__=='__main__':main()
