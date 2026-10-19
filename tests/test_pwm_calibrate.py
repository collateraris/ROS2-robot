"""Protocol and finite-run tests; no UART timing or physical motors emulated."""
import pytest

from room_base.core import frame,parse_frame
from robot_extras.pwm_calibrate import arguments,CalibrationFault,PWMTrial


class Clock:
    now=1.
    def __call__(self):return self.now
    def sleep(self,dt):self.now+=dt


class FirmwarePort:
    def __init__(self,clock,drop_ack=False):
        self.clock=clock;self.drop_ack=drop_ack;self.out=bytearray();self.writes=[]
        self.active=False;self.left=self.right=0;self.nonce=0;self.last_feedback=-1.
        self.start=None;self.stop_time=None

    def reply(self,body):self.out.extend(frame(body))

    @property
    def in_waiting(self):
        if self.clock()-self.last_feedback>=.05:
            self.last_feedback=self.clock()
            self.reply(f'M 1 {self.nonce} {int(self.clock()*1000)} {self.left} {self.right} {int(self.active)}')
        return len(self.out)

    def read(self,n):
        data=bytes(self.out[:n]);del self.out[:n];return data

    def write(self,data):
        fields=parse_frame(data);self.writes.append(fields)
        if fields[0]=='S':
            if self.start is not None:self.stop_time=self.clock()
            self.active=False;self.left=self.right=0;self.reply('D')
        elif fields[0]=='Q':self.reply('V 5 1 120 250')
        elif fields[0]=='H':
            assert not self.active
            self.nonce=int(fields[2]);self.active=True;self.reply(f'R 1 {self.nonce}')
        elif fields[0]=='P':
            assert self.active and int(fields[2])==self.nonce
            self.left,self.right=map(int,fields[4:6])
            if self.start is None:self.start=self.clock()
            if not self.drop_ack:
                self.reply('A '+' '.join(fields[1:])+f' {int(self.clock()*1000)}')
        return len(data)


def runner(drop_ack=False,health=lambda:None,duration=.25):
    clock=Clock();port=FirmwarePort(clock,drop_ack)
    return PWMTrial(port,60,-60,duration,health,clock,clock.sleep),port,clock


def test_bounded_single_trial_acks_and_final_stop():
    trial,port,clock=runner()
    accepted=trial.run()
    assert accepted>=1 and trial.stop_confirmed
    assert not port.active and port.left==port.right==0
    assert port.stop_time-port.start<=.25+.0051
    assert len([x for x in port.writes if x[0]=='H'])==1
    commands=[x for x in port.writes if x[0]=='P']
    assert len(commands)==accepted
    assert all(x[4:6]==['60','-60'] for x in commands)
    assert port.writes[-1]==['S']


def test_missing_ack_stops_early_and_never_retries_motion():
    trial,port,clock=runner(drop_ack=True,duration=1.)
    with pytest.raises(CalibrationFault,match='ACK timeout'):trial.run()
    assert trial.stop_confirmed and not port.active
    assert len([x for x in port.writes if x[0]=='P'])==1
    assert port.stop_time-port.start<=.16


def test_health_loss_and_keyboardinterrupt_always_send_stop():
    calls=0
    def health():
        nonlocal calls
        calls+=1
        if calls>15:raise CalibrationFault('lost scan')
    trial,port,clock=runner(health=health,duration=1.)
    with pytest.raises(CalibrationFault,match='lost scan'):trial.run()
    assert not port.active and port.writes[-1]==['S'] and trial.stop_confirmed
    def interrupted():raise KeyboardInterrupt()
    trial,port,clock=runner(health=interrupted)
    with pytest.raises(KeyboardInterrupt):trial.run()
    assert not port.active and port.writes[-1]==['S']


def test_wrong_protocol_and_corrupt_ack_stop_without_repeat():
    trial,port,clock=runner()
    original=port.write
    def wrong(data):
        result=original(data)
        if parse_frame(data)==['Q']:
            port.out=bytearray(frame('V 4 1 150 250 1000'))
        return result
    port.write=wrong
    with pytest.raises(CalibrationFault,match='unexpected protocol'):trial.run()
    assert not any(x[0]=='P' for x in port.writes)
    assert port.writes[-1]==['S']
    trial,port,clock=runner();original=port.write
    def corrupt(data):
        result=original(data)
        if parse_frame(data)[0]=='P':port.out[-3]^=1
        return result
    port.write=corrupt
    with pytest.raises(ValueError,match='invalid frame'):trial.run()
    assert not port.active and port.writes[-1]==['S']


def test_cli_hard_caps_and_confirmation():
    base=['--port','/dev/explicit','--left-pwm','60','--right-pwm','60']
    with pytest.raises(SystemExit):arguments(base)
    good=base+['--confirm-bench-test']
    assert arguments(good).seconds==.25
    for extra in [['--seconds','1.01'],['--seconds','nan'],['--left-pwm','121'],
                  ['--raised-wheels','--left-pwm','81'],['--raised-wheels','--seconds','.51']]:
        with pytest.raises(SystemExit):arguments(good+extra)
    assert arguments(good+['--raised-wheels','--seconds','.5']).raised_wheels
    for left,right,seconds in [(121,0,.1),(0,0,.1),(60,60,2.),(60,60,float('nan'))]:
        with pytest.raises(ValueError):PWMTrial(object(),left,right,seconds)
