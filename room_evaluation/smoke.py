"""Fabricated deterministic metric smoke; NOT Gazebo/detector experiments."""
import argparse,json,math
from pathlib import Path
from .scenes import generate_scene


def fixture_events():
    scene=generate_scene('dev',0);task=scene['tasks'][0];goal=task['goal']
    for method,status,offset in [('rules','succeeded',0),('llm','succeeded',.8),('oracle_goal','succeeded',0)]:
        eid=f'fixture_{method}'
        base={'episode_id':eid}
        yield dict(base,event='episode_start',scene_id=scene['scene_id'],split='dev',task=task,
                   method=method,source='synthetic_fixture',sim_time=0.,wall_time=1000.)
        for i in range(21):
            fraction=i/20
            yield dict(base,event='pose',sim_time=i*.2,wall_time=1000+i*.2,
                       x=goal['x']-1+fraction+offset,y=goal['y'],yaw=goal['yaw'])
            yield dict(base,event='scan',sim_time=i*.2,wall_time=1000+i*.2,clearance_m=.42)
            yield dict(base,event='contact_sample',source='gazebo_contact',synthetic=True,
                       sim_time=i*.2,wall_time=1000+i*.2,count=0)
        yield dict(base,event='task_result',status=status,sim_time=4.,wall_time=1004.)
        yield dict(base,event='episode_end',reason='task_finished',sim_time=4.,wall_time=1004.)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',default='weeks/week04/synthetic_fixture.jsonl')
    a=p.parse_args(argv);path=Path(a.out);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in fixture_events()),encoding='utf-8')
    print(f'{path}: SYNTHETIC FIXTURE ONLY; no Gazebo or detector was run')


if __name__=='__main__':main()
