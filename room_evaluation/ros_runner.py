"""Simulator-only sequential evaluation. ROS imports are intentionally lazy."""
import argparse
import json
import math
from pathlib import Path
import time
import uuid


def map_goal(scene,goal):
    """Fresh mapping uses initial robot pose as map origin; manifest GT stays world."""
    s=scene['spawn'];dx,dy=goal['x']-s['x'],goal['y']-s['y']
    c,sn=math.cos(s['yaw']),math.sin(s['yaw'])
    return {'x':c*dx+sn*dy,'y':-sn*dx+c*dy,'yaw':goal['yaw']-s['yaw']}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',required=True);p.add_argument('--output',required=True)
    p.add_argument('--method',choices=['rules','llm','oracle_goal'],required=True)
    p.add_argument('--task',default='all');p.add_argument('--enable-sim-actuation',action='store_true')
    p.add_argument('--wall-timeout',type=float,default=300.)
    p.add_argument('--robot-radius',type=float,default=.18)
    p.add_argument('--gt-origin',choices=['world','spawn'],default='world')
    p.add_argument('--sim-node',default='room_sim_drive')
    a=p.parse_args(argv)
    if not a.enable_sim_actuation:p.error('execution requires explicit --enable-sim-actuation; no ARM request was sent')
    if not 0<a.robot_radius<2 or not 5<a.wall_timeout<3600:p.error('invalid physical size or timeout')
    scene=json.loads(Path(a.manifest).read_text(encoding='utf-8'))
    tasks=[t for t in scene['tasks'] if a.task=='all' or t['id']==a.task]
    if not tasks:p.error('task id is absent from manifest')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String,Bool
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Contacts
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    rclpy.init()
    class Runner(Node):
        def __init__(self):
            super().__init__('room_evaluation',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.episode=None;self.latest_pose=None;self.latest_scan=None;self.truth_at=-math.inf;self.planner_config={}
            self.sim_at=-math.inf;self.is_sim=False;self.mode='';self.result=None;self.goal_handle=None;self.armed=False;self.ready_to_arm=False
            path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
            self.stream=path.open('a',encoding='utf-8')
            latch=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool,'/room_sim/is_simulation',self.sim,10)
            self.create_subscription(String,'/room_sim/output_mode',lambda m:setattr(self,'mode',m.data),latch)
            self.create_subscription(Bool,'/room_base/armed',lambda m:setattr(self,'armed',bool(m.data)),latch)
            self.create_subscription(Odometry,'/sim/odom',self.pose,qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
            self.create_subscription(Contacts,'/sim/contacts',self.contacts,qos_profile_sensor_data)
            self.create_subscription(String,'/room_tasks/events',self.task_event,10)
            self.create_subscription(String,'/room_tasks/intervention',lambda m:self.log('intervention',reason=m.data),10)
            self.pub=self.create_publisher(String,'/room_tasks/command',10)
            self.arm=self.create_client(Trigger,'/room_base/arm');self.stop=self.create_client(Trigger,'/room_base/stop')
            self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        def sim_time(self):return self.get_clock().now().nanoseconds*1e-9
        def log(self,event,**data):
            if self.episode is None:return
            row=dict(episode_id=self.episode,event=event,sim_time=self.sim_time(),wall_time=time.monotonic(),**data)
            self.stream.write(json.dumps(row,ensure_ascii=False)+'\n');self.stream.flush()
        def sim(self,m):self.is_sim=bool(m.data);self.sim_at=time.monotonic()
        def pose(self,m):
            q=m.pose.pose.orientation;v=m.pose.pose.position
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            x,y=v.x,v.y
            if a.gt_origin=='spawn':
                s=scene['spawn'];c,sn=math.cos(s['yaw']),math.sin(s['yaw'])
                x,y=s['x']+c*x-sn*y,s['y']+sn*x+c*y;yaw+=s['yaw']
            self.latest_pose=dict(x=x,y=y,yaw=yaw,gt_frame=m.header.frame_id,
                                  source_sim_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
            self.truth_at=time.monotonic();self.log('pose',**self.latest_pose)
        def scan(self,m):
            valid=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
            if valid:
                self.latest_scan=max(0.,min(valid)-a.robot_radius)
                self.log('scan',clearance_m=self.latest_scan)
        def contacts(self,m):
            self.log('contact_sample',source='gazebo_contact',count=len(m.contacts))
            for contact in m.contacts:
                # ros_gz_interfaces Contacts contains Entity collision1/2, with names.
                def name(value):return value if isinstance(value,str) else getattr(value,'name','')
                self.log('contact',collision1=name(contact.collision1),collision2=name(contact.collision2))
        def task_event(self,m):
            try:e=json.loads(m.data)
            except (ValueError,TypeError):return
            if e.get('request_id')!=self.episode:return
            self.log('planner_event',payload=e)
            if e.get('event')=='step' and e.get('status')=='navigating':self.ready_to_arm=True
            if e.get('event')=='task_result':
                self.result=e;self.log('task_result',status=e.get('status',''),reason=e.get('details',{}).get('reason',''))
        def gate(self):
            return (self.get_parameter('use_sim_time').value and self.is_sim and
                    time.monotonic()-self.sim_at<.5 and self.mode=='simulation' and self.sim_time()>0)
        def safe_stop(self,cancel_task=True):
            if self.goal_handle:self.goal_handle.cancel_goal_async()
            if cancel_task and a.method!='oracle_goal' and self.episode:
                self.pub.publish(String(data=json.dumps({'request_id':self.episode,'instruction':'stop'})))
            if self.stop.service_is_ready():return self.stop.call_async(Trigger.Request())
            return None
    node=Runner()
    def spin_until(check,timeout):
        start=time.monotonic()
        while rclpy.ok() and time.monotonic()-start<timeout:
            rclpy.spin_once(node,timeout_sec=.05)
            if check():return True
        return False
    def response(client,request,timeout=5.):
        if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('required ROS service unavailable')
        future=client.call_async(request)
        if not spin_until(future.done,timeout):raise RuntimeError('ROS service response timeout')
        return future.result()
    def check_peer():
        providers=[]
        for name,namespace in node.get_node_names_and_namespaces():
            try:services=node.get_service_names_and_types_by_node(name,namespace)
            except Exception:continue
            if any(n=='/room_base/arm' for n,_ in services):providers.append((name,namespace))
        if providers!=[(a.sim_node,'/')]:raise RuntimeError(f'ARM must have sole simulation provider /{a.sim_node}; got {providers}')
        client=node.create_client(GetParameters,f'/{a.sim_node}/get_parameters')
        req=GetParameters.Request();req.names=['use_sim_time','simulation_autonomy']
        result=response(client,req)
        if len(result.values)!=2 or not all(v.bool_value for v in result.values):
            raise RuntimeError('simulation adapter requires use_sim_time=true and simulation_autonomy=true')
        node.destroy_client(client)
        if a.method!='oracle_goal':
            client=node.create_client(GetParameters,'/room_tasks/get_parameters')
            req=GetParameters.Request();req.names=['backend','enable_execution','shadow_only','use_sim_time','model','use_image','robot_radius_m','standoff_m']
            result=response(client,req);node.destroy_client(client)
            expected='rules' if a.method=='rules' else 'ollama'
            values=result.values
            if len(values)!=8 or values[0].string_value!=expected or not values[1].bool_value or values[2].bool_value or not values[3].bool_value:
                raise RuntimeError('task planner backend/execution/clock does not match evaluation method')
            node.planner_config={'backend':values[0].string_value,'model':values[4].string_value,
                                 'use_image':values[5].bool_value,'robot_radius_m':values[6].double_value,
                                 'standoff_m':values[7].double_value}
            if abs(values[6].double_value-a.robot_radius)>.001:
                raise RuntimeError('planner robot radius must match evaluator --robot-radius')
    try:
        if not spin_until(lambda:node.gate() and node.latest_pose is not None and node.latest_scan is not None,25.):
            raise RuntimeError('simulation/GT/LiDAR readiness unavailable: no ARM sent')
        check_peer()
        if not node.nav.wait_for_server(timeout_sec=10.):raise RuntimeError('Nav2 action server unavailable')
        for task in tasks:
            if not node.gate():raise RuntimeError('simulation gate lost before ARM')
            check_peer()
            if task['criteria']['kind']=='standoff' and a.method!='oracle_goal' and abs(node.planner_config['standoff_m']-task['criteria']['standoff_m'])>.001:
                raise RuntimeError('planner standoff_m must match manifest criterion')
            node.episode=f"{scene['scene_id']}_{task['id']}_{a.method}_{uuid.uuid4().hex[:8]}"
            node.result=None;node.goal_handle=None;node.ready_to_arm=False;arm_attempted=False
            node.log('episode_start',scene_id=scene['scene_id'],split=scene['split'],task=task,
                     method=a.method,source='gazebo',gt_origin=a.gt_origin,gt_frame=scene['gt_frame'],planner_config=node.planner_config,
                     oracle_qualification='navigation diagnostic with GT goal, semantics bypassed' if a.method=='oracle_goal' else '')
            node.log('pose',**node.latest_pose);node.log('scan',clearance_m=node.latest_scan)
            if a.method=='oracle_goal':
                goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
                g=map_goal(scene,task['goal']);goal.pose.pose.position.x=g['x'];goal.pose.pose.position.y=g['y']
                goal.pose.pose.orientation.z=math.sin(g['yaw']/2);goal.pose.pose.orientation.w=math.cos(g['yaw']/2)
                sent=node.nav.send_goal_async(goal)
                if not spin_until(sent.done,5.):raise RuntimeError('oracle goal acceptance timeout')
                node.goal_handle=sent.result()
                if not node.goal_handle.accepted:
                    node.result={'status':'rejected'};node.log('task_result',status='rejected',reason='Nav2 goal rejected')
                else:
                    node.ready_to_arm=True
                    result_future=node.goal_handle.get_result_async()
                    def oracle_result(f):
                        status=f.result().status;node.result={'status':'succeeded' if status==4 else 'failed'}
                        node.log('task_result',status=node.result['status'],reason=f'nav2_status_{status}')
                    result_future.add_done_callback(oracle_result)
            else:node.pub.publish(String(data=json.dumps({'request_id':node.episode,'instruction':task['command']},ensure_ascii=False)))
            start_sim=node.sim_time();start_wall=time.monotonic();last_clock=start_sim;changed=start_wall;reason='task_finished'
            while rclpy.ok():
                rclpy.spin_once(node,timeout_sec=.05);now=node.sim_time();wall=time.monotonic()
                if now>last_clock:last_clock=now;changed=wall
                if not node.gate():reason='simulation_gate_lost';break
                if wall-changed>3.:reason='clock_stalled';break
                if wall-start_wall>a.wall_timeout:reason='wall_timeout';break
                if now-start_sim>task.get('timeout_sim_s',150.):reason='simulation_timeout';break
                if node.result:break
                if node.ready_to_arm and not arm_attempted:
                    # Plan/path and Nav2 acceptance precede ARM. Slow LLM inference
                    # never consumes a motor watchdog grace period.
                    check_peer()
                    if not node.gate():reason='simulation_gate_lost';break
                    arm_attempted=True
                    armed=response(node.arm,Trigger.Request())
                    if not armed.success:
                        node.result={'status':'failed'}
                        node.log('task_result',status='failed',reason='arm_rejected:'+armed.message)
                        reason='arm_rejected';break
                    if not spin_until(lambda:node.armed and node.gate(),2.):
                        reason='arm_confirmation_timeout';break
            if node.result:
                final_at=node.sim_time();spin_until(lambda:node.sim_time()-final_at>=.3,2.)
            node.safe_stop(cancel_task=not node.result or node.result.get('status')!='succeeded');node.log('episode_end',reason=reason);node.episode=None
            spin_until(lambda:not node.armed,2.)
            # Avoid automatic recovery/rearm after failure, timeout, contact/data fault.
            if reason!='task_finished' or not node.result or node.result.get('status')!='succeeded':break
        print(f'Raw Gazebo event log: {a.output}; run room_evaluation.cli for GT validation')
    except KeyboardInterrupt:
        node.safe_stop();node.log('episode_end',reason='user_abort')
    except Exception as exc:
        node.safe_stop();node.log('episode_end',reason='shutdown',detail=str(exc));raise
    finally:
        stop_future=node.safe_stop()
        if stop_future is not None and rclpy.ok():
            try:spin_until(stop_future.done,2.)
            except Exception:pass
        node.stream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
