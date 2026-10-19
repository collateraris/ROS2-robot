"""Exercise serial/ARM callback logic without claiming a ROS middleware test."""
import importlib.util
import math
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from room_base.encoderless_core import OpenLoopMapping, SpeedCurve


def twist(v=0.,w=0.):
    return SimpleNamespace(linear=SimpleNamespace(x=v,y=0.,z=0.),
                           angular=SimpleNamespace(x=0.,y=0.,z=w))


class Stamped:
    def __init__(self):
        self.header=SimpleNamespace(stamp=None,frame_id='')
        self.twist=twist()


@pytest.fixture
def bridge(monkeypatch):
    # Only messages and import-time names are mocked. Methods tested below are
    # the actual encoderless bridge implementation; no executor is emulated.
    names={
        'rclpy': {}, 'rclpy.node': {'Node':object},
        'rclpy.qos': {'qos_profile_sensor_data':None,'QoSProfile':object,
                     'DurabilityPolicy':object,'ReliabilityPolicy':object},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'Twist':object,'TwistStamped':Stamped},
        'nav_msgs': {}, 'nav_msgs.msg': {'Odometry':object},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'LaserScan':object},
        'std_msgs': {}, 'std_msgs.msg': {'Bool':object,'String':object},
        'std_srvs': {}, 'std_srvs.srv': {'Trigger':object},
    }
    for name,attrs in names.items():
        mod=ModuleType(name)
        for k,v in attrs.items():setattr(mod,k,v)
        monkeypatch.setitem(__import__('sys').modules,name,mod)
    name='room_base._encoderless_bridge_test'
    spec=importlib.util.spec_from_file_location(name,Path(__file__).parents[1]/'room_base'/'encoderless_bridge.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    b=mod.EncoderlessBridge.__new__(mod.EncoderlessBridge)
    b.p=dict(max_linear_m_s=.1,max_angular_rad_s=.5,base_frame='base_link',
             command_timeout_s=.25,ack_timeout_s=.15)
    c=SpeedCurve();b.mapping=OpenLoopMapping(c,c,c,c)
    b.boot=1;b.nonce=2;b.mcu_nonce=2;b.seq=0
    b.armed=True;b.pending_arm=None;b.pending={};b.last_output=None
    b.command=None;b.command_at=-math.inf;b.status_text=''
    b.feedback_tick=None;b.feedback_at=-math.inf
    b.estop=False;b.ser=object();b.sent=[]
    b.send=lambda body:b.sent.append(body) or True
    b.publish_state=lambda:None
    b.applied=[];b.applied_alias=[]
    b.applied_pub=SimpleNamespace(publish=b.applied.append)
    b.applied_vel_pub=SimpleNamespace(publish=b.applied_alias.append)
    b.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None))
    monkeypatch.setattr(mod.time,'monotonic',lambda:10.)
    return b


def test_zero_supersedes_nonzero_without_auto_disarm(bridge):
    b=bridge
    b.on_command(twist(.1,0.));b.send_setpoint(10.,b.command)
    old=b.pending[1]
    b.on_command(twist())
    assert b.armed and len(b.pending)==2
    assert b.sent[-1]=='P 1 2 2 0 0'
    # Repeated zeros cannot fill an unbounded ACK queue.
    b.on_command(twist());assert len(b.pending)==2 and b.seq==2
    b.handle(['A','1','2','1',str(old['left']),str(old['right']),'100'],10.01)
    b.handle(['A','1','2','2','0','0','101'],10.02)
    assert b.armed and not b.pending
    assert b.applied[-1].twist.linear.x==b.applied[-1].twist.angular.z==0
    assert len(b.applied)==len(b.applied_alias)==2


def test_health_loss_requires_explicit_rearm_and_new_command(bridge):
    b=bridge;b.on_command(twist(.1,0.));b.send_setpoint(10.,b.command)
    b.on_odom_health(SimpleNamespace(data=False))
    assert not b.armed and not b.pending and b.command is None and b.sent[-1]=='S'
    b.on_odom_health(SimpleNamespace(data=True));b.on_command(twist(.1,0.))
    assert not b.armed and b.command is None


def test_unknown_ack_and_replayed_heartbeat_fail_closed(bridge):
    b=bridge
    with pytest.raises(ValueError):b.handle(['A','1','2','9','60','60','100'],10.)
    b.handle(['M','1','2','100','0','0','1'],10.)
    b.handle(['M','1','2','100','0','0','1'],10.1)
    assert b.feedback_at==10.
    with pytest.raises(ValueError):b.handle(['M','1','2','99','0','0','1'],10.2)
    b.handle(['M','1','2','150','0','0','0'],10.3)
    assert not b.armed and b.sent[-1]=='S'


def test_no_command_cache_before_arm_and_estop_reset_no_resume(bridge):
    b=bridge;b.disarm('test')
    b.on_command(twist(.1,0.));assert b.command is None
    b.on_estop(SimpleNamespace(data=True));assert b.estop
    response=SimpleNamespace(success=None,message='')
    b.on_reset_estop(None,response)
    assert response.success and not b.estop and not b.armed
