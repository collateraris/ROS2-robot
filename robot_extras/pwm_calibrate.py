"""One bounded manual protocol-V5 PWM trial; never invent speed/odometry.

Disable the normal bridge first (start_bridge:=false). Floor trials require the
running LiDAR/RF2O guard. --raised-wheels only checks direction on a secure stand;
it does not establish floor speed calibration. Measurements are made separately.
"""
import argparse
import math
import sys
import time

from room_base.core import frame, fresh, parse_frame, uint32


class CalibrationFault(RuntimeError):
    pass


def arguments(argv=None):
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--port',required=True,help='explicit Arduino /dev/serial/by-id/... path')
    p.add_argument('--left-pwm',type=int,required=True)
    p.add_argument('--right-pwm',type=int,required=True)
    p.add_argument('--seconds',type=float,default=.25)
    p.add_argument('--confirm-bench-test',action='store_true',
                   help='I supervise this one trial with an accessible physical power cut')
    p.add_argument('--raised-wheels',action='store_true',
                   help='secure raised wheels only; omit ROS sensor gates, PWM<=80 and <=0.5s')
    args=p.parse_args(argv)
    if not args.confirm_bench_test:
        p.error('--confirm-bench-test is required for a manual motor trial')
    limit=80 if args.raised_wheels else 120
    duration_limit=.5 if args.raised_wheels else 1.
    if max(abs(args.left_pwm),abs(args.right_pwm))>limit or not (args.left_pwm or args.right_pwm):
        p.error(f'nonzero signed PWM must be within [-{limit}, {limit}]')
    if not math.isfinite(args.seconds) or not 0<args.seconds<=duration_limit:
        p.error(f'--seconds must be within (0, {duration_limit}]')
    return args


class FloorSensors:
    """Read-only ROS gates for supervised floor trials; no motion publishers."""
    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool
        self.rclpy=rclpy
        rclpy.init()
        self.node=rclpy.create_node('pwm_calibration_health')
        self.scan_at=self.odom_at=self.health_at=-math.inf
        self.scan_stamp=self.odom_stamp=None
        self.healthy=False
        self.node.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.node.create_subscription(Bool,'/room_base/odom_healthy',self.on_health,1)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def on_scan(self,msg):
        stamp=self.stamp(msg)
        age=(self.node.get_clock().now().nanoseconds-stamp)/1e9
        if (stamp<=0 or not fresh(age,.30) or not msg.ranges or not any(
                math.isfinite(x) and msg.range_min<=x<=msg.range_max for x in msg.ranges)):
            self.scan_at=-math.inf;return
        if self.scan_stamp is not None and stamp<=self.scan_stamp:return
        self.scan_stamp=stamp;self.scan_at=time.monotonic()

    def on_odom(self,msg):
        stamp=self.stamp(msg)
        age=(self.node.get_clock().now().nanoseconds-stamp)/1e9
        q=msg.pose.pose.orientation
        values=(msg.pose.pose.position.x,msg.pose.pose.position.y,q.x,q.y,q.z,q.w,
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z)
        if (stamp<=0 or not fresh(age,.35) or msg.header.frame_id!='odom' or
                msg.child_frame_id!='base_link' or not all(math.isfinite(x) for x in values) or
                not .95<=sum(x*x for x in (q.x,q.y,q.z,q.w))<=1.05):
            self.odom_at=-math.inf;return
        if self.odom_stamp is not None and stamp<=self.odom_stamp:return
        self.odom_stamp=stamp;self.odom_at=time.monotonic()

    def on_health(self,msg):
        self.healthy=bool(msg.data);self.health_at=time.monotonic()

    def reason(self):
        self.rclpy.spin_once(self.node,timeout_sec=0.)
        now=time.monotonic();ros=self.node.get_clock().now().nanoseconds
        if not self.healthy or not fresh(now-self.health_at,.20):return 'RF2O health false/stale'
        for name,receipt,stamp,limit in [('scan',self.scan_at,self.scan_stamp,.30),
                                        ('odometry',self.odom_at,self.odom_stamp,.35)]:
            if stamp is None or not fresh(now-receipt,limit) or not fresh((ros-stamp)/1e9,limit):
                return f'{name} missing/stale'
        return ''

    def check(self):
        reason=self.reason()
        if reason:raise CalibrationFault(reason)

    def wait_ready(self):
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            if not self.reason():return
            time.sleep(.01)
        raise CalibrationFault(f'floor sensor gates unavailable: {self.reason()}')

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():self.rclpy.shutdown()


class PWMTrial:
    """Single-motion protocol runner, dependency-injected for host tests."""
    def __init__(self,serial_port,left,right,seconds,check_health=lambda:None,
                 clock=time.monotonic,sleep=time.sleep):
        if (type(left) is not int or type(right) is not int or
                not 0<max(abs(left),abs(right))<=120 or
                not math.isfinite(seconds) or not 0<seconds<=1.):
            raise ValueError('trial exceeds hard PWM/duration limits')
        self.ser=serial_port;self.left=left;self.right=right;self.seconds=seconds
        self.check_health=check_health;self.clock=clock;self.sleep=sleep
        self.rx=bytearray();self.boot=None;self.nonce=0;self.seq=0
        self.limit=None;self.mcu_nonce=0;self.feedback_at=-math.inf;self.feedback_tick=None
        self.armed=False;self.pending_arm=False;self.pending={}
        self.stop_confirmed=False;self.stopping=False;self.accepted=0

    def send(self,body):
        data=frame(body)
        if self.ser.write(data)!=len(data):raise CalibrationFault('partial serial write')

    def handle(self,fields):
        now=self.clock();kind=fields[0]
        if kind=='V' and len(fields)==5:
            version,boot,limit,watchdog=map(uint32,fields[1:])
            if version!=5 or not boot or limit<max(abs(self.left),abs(self.right)) or watchdog!=250:
                raise CalibrationFault('wrong firmware: require signed-PWM protocol V5')
            if self.boot is not None and boot!=self.boot:raise CalibrationFault('MCU reboot')
            self.boot=boot;self.limit=limit
        elif kind=='M' and len(fields)==7:
            boot,nonce,tick=map(uint32,fields[1:4]);left,right=int(fields[4]),int(fields[5]);active=uint32(fields[6])
            if self.boot is None:return
            if boot!=self.boot or active not in (0,1) or max(abs(left),abs(right))>120:
                raise CalibrationFault('invalid MCU heartbeat')
            if not active and (left or right):raise CalibrationFault('disarmed MCU nonzero PWM')
            if self.feedback_tick is not None:
                elapsed=(tick-self.feedback_tick)&0xffffffff
                if not elapsed:return
                if elapsed>=0x80000000:raise CalibrationFault('replayed MCU tick')
            self.feedback_tick=tick;self.feedback_at=now;self.mcu_nonce=max(self.mcu_nonce,nonce)
            if self.stopping and not active and not (left or right):self.stop_confirmed=True
            elif self.armed and (not active or nonce!=self.nonce):raise CalibrationFault('MCU watchdog/disarm')
        elif kind=='R' and len(fields)==3:
            boot,nonce=map(uint32,fields[1:])
            if not self.pending_arm or (boot,nonce)!=(self.boot,self.nonce):
                raise CalibrationFault('unexpected ARM ACK')
            self.pending_arm=False;self.armed=True
        elif kind=='A' and len(fields)==7:
            boot,nonce,seq=map(uint32,fields[1:4]);left,right=int(fields[4]),int(fields[5]);uint32(fields[6])
            if (boot,nonce)!=(self.boot,self.nonce) or seq not in self.pending or (left,right)!=(self.left,self.right):
                raise CalibrationFault('unknown/mismatched PWM ACK')
            del self.pending[seq];self.accepted+=1
        elif kind=='D' and len(fields)==1:
            if self.stopping:self.stop_confirmed=True
            elif self.armed:raise CalibrationFault('MCU reported disarmed')
        elif kind=='E' and len(fields)==1:raise CalibrationFault('MCU rejected trial')
        else:raise CalibrationFault('unexpected protocol frame')

    def read(self):
        self.rx.extend(self.ser.read(min(self.ser.in_waiting,4096)))
        if len(self.rx)>4096:raise CalibrationFault('serial backlog')
        for _ in range(32):
            if b'\n' not in self.rx:break
            raw,_,rest=self.rx.partition(b'\n');self.rx=bytearray(rest)
            self.handle(parse_frame(bytes(raw)+b'\n'))
        if len(self.rx)>160:raise CalibrationFault('oversize serial frame')

    def step(self):
        self.check_health();self.read()
        now=self.clock()
        if self.armed and not fresh(now-self.feedback_at,.20):raise CalibrationFault('MCU heartbeat stale')
        if any(now-at>.15 for at in self.pending.values()):raise CalibrationFault('PWM ACK timeout')
        self.sleep(.005)

    def stop(self):
        # STOP may be repeated; motion is never automatically retried.
        self.stopping=True;self.armed=False
        deadline=self.clock()+.30;last_send=-math.inf
        while self.clock()<deadline and not self.stop_confirmed:
            try:
                if self.clock()-last_send>=.05:self.send('S');last_send=self.clock()
                self.read()
            except Exception:
                # A final write attempt still follows a read/CRC failure.
                try:self.send('S')
                except Exception:pass
                break
            self.sleep(.005)

    def run(self):
        try:
            self.send('S');self.send('Q')
            deadline=self.clock()+3.;last_query=self.clock()
            while self.boot is None or not fresh(self.clock()-self.feedback_at,.20):
                if self.clock()>deadline:raise CalibrationFault('firmware/heartbeat probe timeout')
                self.step()
                if self.clock()-last_query>=.5:self.send('Q');last_query=self.clock()
            if self.mcu_nonce>=0xffffffff:raise CalibrationFault('MCU nonce exhausted')
            self.nonce=self.mcu_nonce+1;self.pending_arm=True
            self.send(f'H {self.boot} {self.nonce}');deadline=self.clock()+.15
            while not self.armed:
                if self.clock()>deadline:raise CalibrationFault('ARM ACK timeout')
                self.step()
            started=self.clock();last_send=-math.inf
            while self.clock()-started<self.seconds:
                self.step();now=self.clock()
                # Recheck duration after the polling sleep: never send a motion
                # refresh after the finite trial boundary.
                if now-started>=self.seconds:break
                if not self.pending and now-last_send>=.05:
                    self.seq+=1;self.pending[self.seq]=now
                    self.send(f'P {self.boot} {self.nonce} {self.seq} {self.left} {self.right}')
                    last_send=now
            if not self.accepted:raise CalibrationFault('trial had no accepted PWM command')
            return self.accepted
        finally:
            self.stop()


def main(argv=None):
    args=arguments(argv);sensors=None;ser=None;trial=None
    try:
        if not args.raised_wheels:
            sensors=FloorSensors();sensors.wait_ready()
        import serial
        # Linux exclusivity: the normal serial bridge must already be stopped.
        ser=serial.Serial(args.port,115200,timeout=0,write_timeout=.02,exclusive=True)
        trial=PWMTrial(ser,args.left_pwm,args.right_pwm,args.seconds,
                       sensors.check if sensors else lambda:None)
        accepted=trial.run()
        print(f'Completed ONE {args.seconds:g}s trial; accepted {accepted} PWM commands. '
              'PWM is not a measured speed. Record floor distance/time separately.')
        if not trial.stop_confirmed:
            raise CalibrationFault('STOP sent but not confirmed; cut motor power and inspect')
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'Trial stopped: {exc or "KeyboardInterrupt"}',file=sys.stderr)
        if trial and not trial.stop_confirmed:
            print('STOP confirmation unavailable: use the physical motor power cut.',file=sys.stderr)
        return 1
    finally:
        if ser:ser.close()
        if sensors:sensors.close()


if __name__=='__main__':
    raise SystemExit(main())
