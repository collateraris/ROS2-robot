import copy
from room_evaluation.metrics import evaluate_episode,robot_obstacle_contact
from room_evaluation.smoke import fixture_events


def episode():
    return [e for e in fixture_events() if e['episode_id']=='fixture_rules']


def test_success_requires_ground_truth_not_planner_claim():
    events=episode();assert evaluate_episode(events)['success']
    for e in events:
        if e['event']=='pose':e['x']+=2
    result=evaluate_episode(events)
    assert not result['success'] and result['reason']=='ground_truth_goal_not_reached'


def test_person_standoff_heading_enforced():
    events=episode()
    for e in events:
        if e['event']=='pose':e['yaw']+=1
    assert evaluate_episode(events)['heading_error_rad']>.4
    assert evaluate_episode(events)['outcome']=='failed'


def test_real_contacts_not_lidar_proximity_decide_collision():
    events=episode()
    for e in events:
        if e['event']=='scan':e['clearance_m']=0.
    assert evaluate_episode(events)['success']
    events.insert(-1,dict(episode_id='fixture_rules',event='contact',collision1='room_robot::base::collision',
                          collision2='wall_n::link::collision',sim_time=4.))
    assert evaluate_episode(events)['reason']=='collision'
    assert not robot_obstacle_contact('room_robot::wheel::collision','ground::link::collision')
    assert not robot_obstacle_contact('room_robot::base::collision','room_robot::wheel::collision')


def test_missing_monitor_or_final_pose_is_incomplete():
    events=[e for e in episode() if e['event']!='contact_sample']
    assert evaluate_episode(events)['outcome']=='incomplete'
    events=episode()
    events=[e for e in events if e['event']!='pose' or e['sim_time']<2.]
    assert 'fresh_final_ground_truth' in evaluate_episode(events)['reason']


def test_monitor_dropout_is_not_false_success():
    events=[e for e in episode() if e['event']!='contact_sample' or e['sim_time']<1.]
    assert 'contact_coverage_gap' in evaluate_episode(events)['reason']


def test_aborted_and_timeout_distinct_and_sources_retained():
    events=episode();events[-1]['reason']='user_abort'
    assert evaluate_episode(events)['outcome']=='aborted'
    events[-1]['reason']='simulation_timeout'
    result=evaluate_episode(events)
    assert result['outcome']=='failed' and result['reason']=='simulation_timeout'
    assert result['source']=='synthetic_fixture'
    assert result['elapsed_sim_s']==4


def test_no_result_cannot_be_success():
    events=[e for e in episode() if e['event']!='task_result']
    assert evaluate_episode(events)['outcome']=='incomplete'


def test_teleport_invalidates_path_and_success():
    events=episode()
    poses=[e for e in events if e['event']=='pose'];poses[5]['x']+=5
    assert 'monotonic_continuous_ground_truth' in evaluate_episode(events)['reason']


def test_floor_contact_heartbeat_does_not_count_collision_and_repeats_deduplicate():
    events=episode()
    for sample in [e for e in events if e['event']=='contact_sample']:
        sample['count']=1
        events.insert(-1,dict(episode_id='fixture_rules',event='contact',
            collision1='room_robot::left_wheel::wheel_collision',
            collision2='ground::link::collision',sim_time=sample['sim_time']))
    assert evaluate_episode(events)['success']
    assert evaluate_episode(events)['collision_pairs']==0
    for i in range(20):
        events.insert(-1,dict(episode_id='fixture_rules',event='contact',
            collision1='room_robot::base_link::contact_bumper',
            collision2='box_0::link::collision',sim_time=3.+i*.01))
    result=evaluate_episode(events)
    assert result['reason']=='collision' and result['collision_pairs']==1
