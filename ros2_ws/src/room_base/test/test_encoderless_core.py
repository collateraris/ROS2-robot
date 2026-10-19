import math
import pytest

from room_base.encoderless_core import OpenLoopMapping, SpeedCurve, SafetyState
from room_base.core import frame, parse_frame


def mapping():
    c=SpeedCurve()
    return OpenLoopMapping(c,c,c,c)


def test_zero_subminimum_and_sustained_minimum():
    c=SpeedCurve(35.,150.,60,120)
    assert c.convert(0.) == (0,0.)
    assert c.convert(34.99) == (0,0.)
    assert c.convert(35.) == (60,35.)
    assert c.convert(150.) == (120,150.)
    assert c.convert(1000.) == (120,150.)
    with pytest.raises(ValueError): c.convert(math.nan)
    with pytest.raises(ValueError): c.convert(-1.)


def test_signed_wheels_and_curvature_saturation():
    m=mapping()
    assert m.convert(0.,0.) == (0,0,0.,0.)
    lp,rp,v,w=m.convert(.1,0.)
    assert lp==rp and 60<=lp<=120 and v==pytest.approx(.1,abs=.002) and w==0
    lp,rp,v,w=m.convert(0.,.5)
    assert lp<0<rp and v==0 and w==pytest.approx(.5,abs=.03)
    lp,rp,v,w=m.convert(100.,0.)
    assert lp==rp==120 and v==.15 and w==0
    lp,rp,v,w=m.convert(-.1,0.)
    assert lp==rp<0 and v<0 and w==0
    # No automatic breakaway pulse for an unachievable tiny turn.
    assert m.convert(0.,.1) == (0,0,0.,0.)


def test_independent_direction_calibration_and_quantisation():
    m=OpenLoopMapping(SpeedCurve(35.,150.,60,120),SpeedCurve(35.,150.,70,120),
                      SpeedCurve(35.,150.,80,120),SpeedCurve(35.,150.,90,120))
    lp,rp,v,w=m.convert(.1,0.)
    assert lp != rp and v==pytest.approx(.1,abs=.002)
    assert abs(w)<.02
    lp,rp,v,w=m.convert(-.1,0.)
    assert abs(lp) != abs(rp) and v<0


@pytest.mark.parametrize('args',[(0.,150.,60,120),(35.,35.,60,120),
                                (35.,150.,0,120),(35.,150.,120,60),
                                (35.,150.,60,256),(math.inf,150.,60,120)])
def test_bad_calibration_rejected(args):
    with pytest.raises(ValueError): SpeedCurve(*args)


def limits():
    return dict(deadman_timeout_s=.25,scan_timeout_s=.3,odom_timeout_s=.35,
                health_timeout_s=.2,feedback_timeout_s=.2)


def healthy():
    return SafetyState(True,True,False,True,True,True,True,
                       deadman_at=10.,scan_at=10.,scan_age=.05,
                       odom_at=10.,odom_age=.05,health_at=10.,feedback_at=10.)


def test_gate_requires_every_independent_source():
    assert healthy().reason(10.1,limits()) == ''
    for key in ['allow_actuation','calibration_confirmed','connected','boot_known',
                'deadman','odom_healthy']:
        state=healthy();setattr(state,key,False)
        assert state.reason(10.1,limits())
    state=healthy();state.estop=True;assert state.reason(10.1,limits())
    for key in ['deadman_at','scan_at','odom_at','health_at','feedback_at']:
        state=healthy();setattr(state,key,9.)
        assert state.reason(10.1,limits())
    for key in ['scan_age','odom_age']:
        state=healthy();setattr(state,key,1.)
        assert state.reason(10.1,limits())
        state=healthy();setattr(state,key,-.01)
        assert state.reason(10.1,limits())


def test_wire_protocol_uses_pwm_identity():
    assert parse_frame(frame('P 12 8 3 -60 120')) == ['P','12','8','3','-60','120']
    bad=bytearray(frame('P 12 8 3 -60 120'));bad[0]=ord('C')
    with pytest.raises(ValueError):parse_frame(bytes(bad))
