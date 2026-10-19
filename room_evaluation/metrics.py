"""GT episode metrics. Nav2/action success alone can never satisfy success."""
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def goal_error(task, pose):
    c=task['criteria'];goal=task['goal']
    if c['kind']=='standoff':
        target=c['target'];dx,dy=target['x']-pose['x'],target['y']-pose['y']
        radial=math.hypot(dx,dy)
        return abs(radial-c['standoff_m']),abs(wrap(math.atan2(dy,dx)-pose['yaw']))
    return math.hypot(goal['x']-pose['x'],goal['y']-pose['y']),abs(wrap(goal['yaw']-pose['yaw']))


def robot_obstacle_contact(a,b):
    """Exclude robot-ground/self contacts; do not infer collisions from LiDAR/odom."""
    a,b=str(a).lower(),str(b).lower()
    robot=lambda x:'room_robot' in x
    ground=lambda x:any(part in ('ground','ground_plane','floor') for part in x.replace('/','::').split('::'))
    return (robot(a)!=robot(b)) and not ground(a) and not ground(b)


def evaluate_episode(events):
    starts=[e for e in events if e.get('event')=='episode_start']
    if len(starts)!=1:raise ValueError('each episode needs exactly one episode_start')
    start=starts[0];task=start['task'];end=next((e for e in reversed(events) if e.get('event')=='episode_end'),None)
    poses=[e for e in events if e.get('event')=='pose']
    contact_samples=[e for e in events if e.get('event')=='contact_sample' and e.get('source')=='gazebo_contact']
    lidar=[float(e['clearance_m']) for e in events if e.get('event')=='scan' and
           isinstance(e.get('clearance_m'),(int,float)) and math.isfinite(e['clearance_m'])]
    contacts={tuple(sorted((e.get('collision1',''),e.get('collision2','')))) for e in events
              if e.get('event')=='contact' and robot_obstacle_contact(e.get('collision1',''),e.get('collision2',''))}
    results=[e for e in events if e.get('event')=='task_result']
    interventions=sum(e.get('event')=='intervention' for e in events)
    missing=[]
    if end is None:missing.append('episode_end')
    if len(poses)<2:missing.append('ground_truth_trajectory')
    if len(contact_samples)<2:missing.append('gazebo_contact_stream')
    if not lidar:missing.append('lidar_clearance')
    finite_poses=all(all(isinstance(p.get(k),(int,float)) and math.isfinite(p[k]) for k in
                         ('x','y','yaw','sim_time')) for p in poses)
    if not finite_poses:missing.append('finite_ground_truth')
    if any(e.get('event')=='contact' and not (e.get('collision1') and e.get('collision2')) for e in events):
        missing.append('resolved_contact_entity_names')
    if start.get('gt_frame') and any(p.get('gt_frame')!=start['gt_frame'] for p in poses):
        missing.append('expected_ground_truth_frame')
    if end:
        for label,samples,limit in [('contact',contact_samples,1.),
                ('lidar',[e for e in events if e.get('event')=='scan'],.6)]:
            times=[e['sim_time'] for e in samples if isinstance(e.get('sim_time'),(int,float))]
            if times:
                times=[start['sim_time']]+times+[end['sim_time']]
                if any(b<a or b-a>limit for a,b in zip(times,times[1:])):
                    missing.append(label+'_coverage_gap')
    path=0.
    if finite_poses:
        for a,b in zip(poses,poses[1:]):
            dt=b.get('source_sim_time',b['sim_time'])-a.get('source_sim_time',a['sim_time']);distance=math.hypot(b['x']-a['x'],b['y']-a['y'])
            if dt<0 or distance>max(.5,3.*max(dt,0)):
                missing.append('monotonic_continuous_ground_truth');break
            path+=distance
    if poses and end and finite_poses:
        if not -.05<=end['sim_time']-poses[-1].get('source_sim_time',poses[-1]['sim_time'])<=.5:missing.append('fresh_final_ground_truth')
        if end['sim_time']<start['sim_time']:missing.append('monotonic_simulation_clock')
    end_reason=end.get('reason','') if end else 'log_truncated'
    position_error=yaw_error=None
    if poses and finite_poses:position_error,yaw_error=goal_error(task,poses[-1])
    c=task['criteria'];position_tol=c.get('distance_tolerance_m',c.get('position_tolerance_m',.25))
    physical_success=position_error is not None and position_error<=position_tol and yaw_error<=c['yaw_tolerance_rad']
    result_status=results[-1].get('status','') if results else ''
    if end_reason in ('user_abort','shutdown','simulation_gate_lost','clock_stalled','wall_timeout'):
        outcome='aborted';reason=end_reason
    elif missing:
        outcome='incomplete';reason='missing_or_invalid:'+','.join(dict.fromkeys(missing))
    elif end_reason in ('timeout','simulation_timeout'):
        outcome='failed';reason='simulation_timeout'
    elif not results:
        outcome='incomplete';reason='task_result_missing'
    elif result_status!='succeeded':
        outcome='failed';reason=results[-1].get('reason') or f'task_{result_status}'
    elif contacts:
        outcome='failed';reason='collision'
    elif interventions:
        outcome='failed';reason='operator_intervention'
    elif not physical_success:
        outcome='failed';reason='ground_truth_goal_not_reached'
    else:outcome='success';reason='ground_truth_verified'
    return {'episode_id':start['episode_id'],'scene_id':start['scene_id'],'split':start.get('split','unknown'),
            'task_id':task['id'],'method':start['method'],'source':start.get('source','unknown'),
            'outcome':outcome,'reason':reason,'success':outcome=='success','physical_goal_reached':physical_success,
            'position_or_standoff_error_m':position_error,'heading_error_rad':yaw_error,
            'path_length_gt_m':path if finite_poses else None,
            'elapsed_sim_s':end['sim_time']-start['sim_time'] if end else None,
            'elapsed_wall_s':end['wall_time']-start['wall_time'] if end else None,
            'collision_pairs':len(contacts),'contact_samples':len(contact_samples),
            'min_lidar_clearance_approx_m':min(lidar) if lidar else None,
            'interventions':interventions,'task_result':result_status}


def read_episodes(paths):
    groups=defaultdict(list)
    for path in paths:
        for n,line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(),1):
            if not line.strip():continue
            event=json.loads(line)
            if not event.get('episode_id'):raise ValueError(f'{path}:{n}: episode_id required')
            groups[event['episode_id']].append(event)
    return [evaluate_episode(events) for events in groups.values()]


def report(rows,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if not rows:raise ValueError('no episodes')
    with (out/'episodes.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    summary=[]
    groups=defaultdict(list)
    for row in rows:groups[(row['source'],row['split'],row['method'])].append(row)
    for (source,split,method),group in sorted(groups.items()):
        evaluated=[r for r in group if r['outcome'] in ('success','failed')]
        summary.append({'source':source,'split':split,'method':method,'episodes':len(group),
                        'complete_episodes':len(evaluated),'successes':sum(r['success'] for r in evaluated),
                        'success_rate_complete_only':sum(r['success'] for r in evaluated)/len(evaluated) if evaluated else None,
                        'aborted':sum(r['outcome']=='aborted' for r in group),
                        'incomplete':sum(r['outcome']=='incomplete' for r in group)})
    (out/'summary.json').write_text(json.dumps({'schema_version':1,'groups':summary,
        'limitations':['LiDAR clearance is minimum finite ray minus configured robot enclosing radius; approximate, no contact proof.',
                       'oracle_goal bypasses semantics and uses evaluator target: navigation upper-bound diagnostic, not fair end-to-end semantic competitor.',
                       'Compare rules/llm only with identical scene/task/perception/config; simulator assets are primitive geometry proxies.',
                       'synthetic_fixture rows are fabricated smoke fixtures, never simulator or real detection performance.']},indent=2)+'\n',encoding='utf-8')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:raise RuntimeError('matplotlib required to generate report figure') from exc
    labels=[f"{s['source']}\n{s['split']} / {s['method']}" for s in summary]
    rates=[s['success_rate_complete_only'] or 0 for s in summary]
    fig,axes=plt.subplots(1,2,figsize=(max(9,len(labels)*1.3),4.2))
    axes[0].bar(labels,rates,color='#3c7d9b');axes[0].set_ylim(0,1);axes[0].set_ylabel('Success / complete episodes')
    axes[0].tick_params(axis='x',labelsize=8)
    for outcome,color in [('success','#59976b'),('failed','#b6725c'),('aborted','#a0a0a0'),('incomplete','#d4b158')]:
        vals=[sum(r['outcome']==outcome for r in groups[(s['source'],s['split'],s['method'])]) for s in summary]
        bottom=[sum(r['outcome'] in ['success','failed','aborted','incomplete'][:['success','failed','aborted','incomplete'].index(outcome)]
                    for r in groups[(s['source'],s['split'],s['method'])]) for s in summary]
        axes[1].bar(labels,vals,bottom=bottom,label=outcome,color=color)
    axes[1].set_ylabel('Episodes (including missing/aborted)');axes[1].legend(fontsize=8);axes[1].tick_params(axis='x',labelsize=8)
    title='SYNTHETIC FIXTURE SMOKE — NOT GAZEBO RESULTS' if all(r['source']=='synthetic_fixture' for r in rows) else 'Logged evaluation — sources kept separate'
    fig.suptitle(title);fig.tight_layout();fig.savefig(out/'evaluation.png',dpi=150);plt.close(fig)
    return out
