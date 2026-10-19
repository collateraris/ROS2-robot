#!/usr/bin/env python3
"""Read-only environment checks; does not publish goals or motor commands."""
import argparse
import importlib
import json
from pathlib import Path
import shutil
import sys
from urllib.request import urlopen


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--sim',action='store_true')
    parser.add_argument('--perception',action='store_true')
    parser.add_argument('--ollama',action='store_true')
    args=parser.parse_args()
    checks=[]
    def add(name,ok,detail=''):
        checks.append(dict(name=name,ok=bool(ok),detail=str(detail)))
    add('Python3.12',sys.version_info[:2]==(3,12),sys.version.split()[0])
    names=['numpy','yaml','PIL']
    project_root=Path(__file__).resolve().parents[1]
    names += [name for name in ('room_semantics','room_planning','room_evaluation') if (project_root/name).is_dir()]
    if args.ros or args.sim:names+=['rclpy','room_base.core']
    if args.perception:names+=['torch','transformers']
    for name in names:
        try:
            module=importlib.import_module(name)
            add(name,True,getattr(module,'__version__','importable'))
            if name=='transformers':add('Transformers pinned4.52.3',module.__version__=='4.52.3',module.__version__)
            if name=='torch':add('CUDA availability (CPU is allowed)',True,module.cuda.is_available())
        except Exception as exc:add(name,False,type(exc).__name__+': '+str(exc)[:180])
    if args.ros or args.sim:
        add('ROS Jazzy setup',Path('/opt/ros/jazzy/setup.bash').is_file())
        add('ros2 command',shutil.which('ros2') is not None)
    if args.sim:add('gz command',shutil.which('gz') is not None)
    if args.ollama:
        try:
            with urlopen('http://127.0.0.1:11434/api/tags',timeout=3) as response:body=json.load(response)
            models=[x.get('name') for x in body.get('models',[])]
            add('Local Ollama',True,models)
            add('Qwen2.5VL7B available','qwen2.5vl:7b' in models)
        except Exception as exc:add('Local Ollama',False,type(exc).__name__+': '+str(exc)[:180])
    print(json.dumps({'checks':checks,'ready':all(c['ok'] for c in checks)},ensure_ascii=False,indent=2))
    return 0 if all(c['ok'] for c in checks) else 1


if __name__=='__main__':raise SystemExit(main())
