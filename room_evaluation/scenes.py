"""Deterministic primitive Gazebo fixtures, not a semantic recognition benchmark."""
import argparse
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

SPLITS = {'train': (100, (5.5, 6.5), (4.5, 5.5)),
          'dev': (10000, (7., 7.5), (5.5, 6.)),
          'test': (20000, (8., 9.), (6.5, 7.5))}


def generate_scene(split='train', index=0):
    if split not in SPLITS or not 0 <= index < 1000:
        raise ValueError('split train/dev/test; index in [0,999]')
    base, wr, hr = SPLITS[split]
    seed = base + index
    rng = random.Random(seed)
    width, height = round(rng.uniform(*wr), 3), round(rng.uniform(*hr), 3)
    spawn = {'x': -width/2+1., 'y': -height/2+1., 'yaw': .15}
    person = {'id': 'person_proxy', 'label': 'person', 'x': width/2-1.1,
              'y': round(rng.uniform(.3, height/2-1.1), 3), 'radius': .18}
    table = {'id': 'table_proxy', 'label': 'table', 'x': .1,
             'y': height/2-.8, 'size': [.9, .6, .72]}
    dx, dy = person['x']-spawn['x'], person['y']-spawn['y']
    norm = math.hypot(dx,dy)
    person_goal = {'x': person['x']-dx/norm, 'y': person['y']-dy/norm,
                   'yaw': math.atan2(dy,dx)}
    table_goal = {'x': table['x'], 'y': table['y']-.9, 'yaw': math.pi/2}
    home_goal = dict(spawn)
    tasks = [
        {'id': 'approach_person', 'command': 'подъедь к человеку', 'target_id': person['id'],
         'goal': person_goal, 'timeout_sim_s': 150.,
         'criteria': {'kind':'standoff', 'target': {'x':person['x'],'y':person['y']},
                      'standoff_m':1., 'distance_tolerance_m':.25, 'yaw_tolerance_rad':.4}},
        {'id':'go_table', 'command':'поезжай к table', 'semantic_example':'подъедь к столу', 'target_id':table['id'],
         'goal':table_goal, 'timeout_sim_s':150.,
         'criteria':{'kind':'pose', 'position_tolerance_m':.25,'yaw_tolerance_rad':.4}},
        {'id':'go_home', 'command':'домой', 'target_id':'home', 'goal':home_goal,
         'timeout_sim_s':150.,
         'criteria':{'kind':'pose','position_tolerance_m':.25,'yaw_tolerance_rad':.4}}]
    protected = [(spawn['x'],spawn['y']), (person['x'],person['y']),
                 (table['x'],table['y'])]+[(t['goal']['x'],t['goal']['y']) for t in tasks]
    obstacles = []
    for attempt in range(200):
        if len(obstacles) == 3: break
        x,y = rng.uniform(-width/2+.7,width/2-.7), rng.uniform(-height/2+.7,height/2-.7)
        if min(math.hypot(x-px,y-py) for px,py in protected) < .85: continue
        if any(math.hypot(x-o['x'],y-o['y']) < .9 for o in obstacles): continue
        obstacles.append({'id':f'box_{len(obstacles)}','label':'box','x':round(x,3),
                          'y':round(y,3),'size':[.35,.45,.5]})
    return {'schema_version':1,'scene_id':f'{split}_{index:03d}','split':split,'seed':seed,
            'benchmark_kind':'primitive_fixture_not_zero_shot_semantics',
            'world_name':'room','gt_frame':'sim_world','room':{'width':width,'height':height},
            'spawn':spawn,'objects':[person,table]+obstacles,'tasks':tasks,
            'notes':['Primitive person/table shapes are geometry proxies; real detector recognition is not assumed.',
                     'GT objects/goals are evaluator-only; no GT-to-perception publisher exists.',
                     'Known table/home map waypoints are an explicit fixture prior, not semantic grounding.']}


def _node(parent, tag, text=None, **attrs):
    value=ET.SubElement(parent,tag,attrs)
    if text is not None: value.text=str(text)
    return value


def world_sdf(scene):
    root=ET.Element('sdf',version='1.9'); world=_node(root,'world',name='room')
    _node(world,'gravity','0 0 -9.81')
    physics=_node(world,'physics',name='physics',type='ignored')
    _node(physics,'max_step_size','.001');_node(physics,'real_time_factor','1.0')
    for filename,name in [('gz-sim-physics-system','gz::sim::systems::Physics'),
                          ('gz-sim-user-commands-system','gz::sim::systems::UserCommands'),
                          ('gz-sim-scene-broadcaster-system','gz::sim::systems::SceneBroadcaster'),
                          ('gz-sim-sensors-system','gz::sim::systems::Sensors'),
                          ('gz-sim-contact-system','gz::sim::systems::Contact')]:
        p=_node(world,'plugin',filename=filename,name=name)
        if 'sensors' in filename:_node(p,'render_engine','ogre2')
    light=_node(world,'light',name='sun',type='directional');_node(light,'pose','0 0 8 0 0 0')
    _node(light,'direction','-.5 -.3 -1');_node(light,'diffuse','.9 .9 .9 1')
    def box(name,x,y,z,sizes,color):
        model=_node(world,'model',name=name);_node(model,'static','true')
        _node(model,'pose',f'{x} {y} {z} 0 0 0'); link=_node(model,'link',name='link')
        for kind in ('collision','visual'):
            body=_node(link,kind,name=kind);geo=_node(body,'geometry')
            b=_node(geo,'box');_node(b,'size',' '.join(map(str,sizes)))
            if kind=='visual':
                mat=_node(body,'material');_node(mat,'ambient',color);_node(mat,'diffuse',color)
    w,h=scene['room']['width'],scene['room']['height']
    box('ground',0,0,-.05,[w+1,h+1,.1],'.65 .65 .65 1')
    box('wall_n',0,h/2,1.25,[w+.2,.15,2.5],'.8 .8 .8 1')
    box('wall_s',0,-h/2,1.25,[w+.2,.15,2.5],'.8 .8 .8 1')
    box('wall_e',w/2,0,1.25,[.15,h,2.5],'.8 .8 .8 1')
    box('wall_w',-w/2,0,1.25,[.15,h,2.5],'.8 .8 .8 1')
    for obj in scene['objects']:
        if obj['label']=='person':
            model=_node(world,'model',name=obj['id']);_node(model,'static','true')
            _node(model,'pose',f"{obj['x']} {obj['y']} .825 0 0 0")
            link=_node(model,'link',name='link')
            for kind in ('collision','visual'):
                body=_node(link,kind,name=kind);geo=_node(body,'geometry');cyl=_node(geo,'cylinder')
                _node(cyl,'radius',obj['radius']);_node(cyl,'length','1.65')
                if kind=='visual':
                    mat=_node(body,'material');_node(mat,'ambient','.15 .25 .8 1');_node(mat,'diffuse','.15 .25 .8 1')
        else:
            box(obj['id'],obj['x'],obj['y'],obj['size'][2]/2,obj['size'],'.4 .25 .1 1')
    inc=_node(world,'include');_node(inc,'uri','model://room_robot');_node(inc,'name','room_robot')
    p=scene['spawn'];_node(inc,'pose',f"{p['x']} {p['y']} .034 0 0 {p['yaw']}")
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')+'\n'


def save_scene(scene,directory):
    dest=Path(directory)/scene['scene_id'];dest.mkdir(parents=True,exist_ok=True)
    (dest/'manifest.json').write_text(json.dumps(scene,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (dest/'room.sdf').write_text(world_sdf(scene),encoding='utf-8')
    # This public prior intentionally includes only user-configured named map points.
    # It does not export person location or a synthetic semantic graph.
    from .ros_runner import map_goal
    points={t['target_id'].replace('_proxy',''):{'pose':[map_goal(scene,t['goal'])[a] for a in ('x','y','yaw')],
            'aliases':(['стол','столу'] if t['id']=='go_table' else ['домой','дом'])}
            for t in scene['tasks'] if t['id'] in ('go_table','go_home')}
    try:
        import yaml
        (dest/'known_waypoints.yaml').write_text(yaml.safe_dump({'frame_id':'map','waypoints':points},
            allow_unicode=True,sort_keys=False),encoding='utf-8')
    except ImportError: pass
    return dest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default='weeks/week04/generated');parser.add_argument('--count',type=int,default=3)
    parser.add_argument('--split',choices=['all',*SPLITS],default='all');args=parser.parse_args(argv)
    if not 1<=args.count<=1000:parser.error('count must be 1..1000')
    for split in (SPLITS if args.split=='all' else [args.split]):
        for i in range(args.count):print(save_scene(generate_scene(split,i),args.out))


if __name__=='__main__':main()
