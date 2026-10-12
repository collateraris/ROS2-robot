import asyncio
import sys
import tempfile
import unittest
from pathlib import Path

from aiohttp import CookieJar
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_web_teleop'))
from robot_web_teleop.web_server import RobotServer


class FakeIO:
    def __init__(self):
        self.ready = self.camera = self.video_ok = True
        self.outputs = []
        self.source = 'idle'
        self.emergency_stops = 0

    def health(self):
        return {'drive_ready': self.ready, 'camera_fresh': self.camera,
                'drive_source': self.source}

    def frame(self):
        return (b'jpeg-test-bytes', 'frame-1') if self.camera else (None, '')

    def frame_fresh(self, token):
        return self.video_ok and self.camera and token == 'frame-1'

    def publish(self, linear, angular):
        self.outputs.append((linear, angular))

    def emergency_stop(self):
        self.emergency_stops += 1


class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        page = Path(self.tmp.name) / 'index.html'
        page.write_text('<html>test</html>')
        self.io = FakeIO()
        self.gateway = RobotServer(self.io, page, 'test-password')
        self.client = TestClient(TestServer(self.gateway.app),
                                 cookie_jar=CookieJar(unsafe=True))
        await self.client.start_server()
        self.origin = str(self.client.make_url('')).rstrip('/')

    async def asyncTearDown(self):
        await self.client.close()
        self.tmp.cleanup()

    async def login(self):
        r = await self.client.post('/api/login', json={'password': 'test-password'},
                                   headers={'Origin': self.origin})
        self.assertEqual(r.status, 200)
        self.assertTrue(r.cookies['robot_session']['httponly'])
        self.assertEqual(r.cookies['robot_session']['samesite'], 'Strict')

    async def connect(self):
        await self.login()
        ws = await self.client.ws_connect('/ws', headers={'Origin': self.origin})
        state = await ws.receive_json(timeout=1)
        return ws, state

    async def arm(self, ws, state):
        await ws.send_json(dict(type='arm', challenge=state['challenge'], frame='frame-1'))
        state = await ws.receive_json(timeout=1)
        self.assertTrue(state['armed'])
        self.assertTrue(state['owner'])
        return state

    async def test_auth_and_cross_origin(self):
        for route in ['/api/status', '/api/frame.jpg']:
            self.assertEqual((await self.client.get(route)).status, 401)
        r = await self.client.post('/api/login', json={'password': 'test-password'},
                                   headers={'Origin': 'http://other-host'})
        self.assertEqual(r.status, 403)
        await self.login()
        self.assertEqual((await self.client.get('/api/status')).status, 200)
        r = await self.client.get('/api/frame.jpg')
        self.assertEqual(r.headers['X-Frame-Token'], 'frame-1')
        self.assertEqual(r.headers['Cache-Control'], 'no-store')
        self.assertEqual(await r.read(), b'jpeg-test-bytes')

    async def test_motion_then_video_link_failure(self):
        ws, state = await self.connect()
        state = await self.arm(ws, state)
        await ws.send_json(dict(type='drive', challenge=state['challenge'], frame='frame-1',
                                seq=1, linear=0.4, angular=0.2))
        await asyncio.sleep(0.06)
        self.assertIn((0.4, 0.2), self.io.outputs)
        self.io.video_ok = False  # Camera and WS are alive, phone frame ACK is stale.
        state = await ws.receive_json(timeout=1)
        await ws.send_json(dict(type='drive', challenge=state['challenge'], frame='frame-1',
                                seq=2, linear=0.4, angular=0.2))
        await asyncio.sleep(0.07)
        self.assertFalse(self.gateway.control.armed)
        self.assertEqual(self.io.outputs[-1], (0, 0))
        await ws.close()

    async def test_disconnect_and_reconnect_never_rearms(self):
        ws, state = await self.connect()
        await self.arm(ws, state)
        await ws.close()
        await asyncio.sleep(0.03)
        self.assertFalse(self.gateway.control.armed)
        ws = await self.client.ws_connect('/ws', headers={'Origin': self.origin})
        state = await ws.receive_json(timeout=1)
        self.assertFalse(state['armed'])
        await ws.close()

    async def test_deadman_without_client_packets(self):
        ws, state = await self.connect()
        await self.arm(ws, state)
        await asyncio.sleep(0.3)
        self.assertFalse(self.gateway.control.armed)
        self.assertEqual(self.io.outputs[-1], (0, 0))
        await ws.close()

    async def test_hardware_fault_blocks_arm(self):
        ws, state = await self.connect()
        self.io.ready = False
        await ws.send_json(dict(type='arm', challenge=state['challenge'], frame='frame-1'))
        await asyncio.sleep(0.03)
        self.assertFalse(self.gateway.control.armed)
        await ws.close()

    async def test_direct_sight_accepts_control_without_camera(self):
        self.gateway.camera_required = False
        self.io.camera = self.io.video_ok = False
        ws, state = await self.connect()
        self.assertFalse(state['camera_required'])
        self.assertFalse(state['camera_fresh'])
        state = await self.arm(ws, state)
        await ws.send_json(dict(type='drive', challenge=state['challenge'], frame='',
                                seq=1, linear=.4, angular=.2))
        await asyncio.sleep(.06)
        self.assertIn((.4, .2), self.io.outputs)
        self.io.ready = False
        await asyncio.sleep(.06)
        self.assertFalse(self.gateway.control.armed)
        self.assertEqual(self.io.outputs[-1], (0, 0))
        self.assertEqual(self.io.emergency_stops, 0)
        await ws.close()

    async def test_camera_required_still_blocks_missing_camera(self):
        self.io.camera = False
        ws, state = await self.connect()
        self.assertTrue(state['camera_required'])
        await ws.send_json(dict(type='arm', challenge=state['challenge'], frame='frame-1'))
        await asyncio.sleep(.03)
        self.assertFalse(self.gateway.control.armed)
        await ws.close()

    async def test_bluetooth_takeover_cancels_wifi_without_global_stop(self):
        ws, state = await self.connect()
        await self.arm(ws, state)
        self.io.source = 'bluetooth'
        await asyncio.sleep(.07)
        self.assertFalse(self.gateway.control.armed)
        await ws.send_json({'type': 'release'})
        await asyncio.sleep(.03)
        self.assertEqual(self.io.emergency_stops, 0)
        await ws.send_json({'type': 'stop'})
        await asyncio.sleep(.03)
        self.assertEqual(self.io.emergency_stops, 1)
        await ws.close()

    async def test_nonowner_release_does_not_cancel_owner(self):
        ws, state = await self.connect()
        await self.arm(ws, state)
        viewer = await self.client.ws_connect('/ws', headers={'Origin': self.origin})
        await viewer.receive_json(timeout=1)
        await viewer.send_json({'type': 'release'})
        await asyncio.sleep(.03)
        self.assertTrue(self.gateway.control.armed)
        self.assertEqual(self.io.emergency_stops, 0)
        await viewer.send_json({'type': 'stop'})
        await asyncio.sleep(.03)
        self.assertFalse(self.gateway.control.armed)
        self.assertEqual(self.io.emergency_stops, 1)
        await viewer.close()
        await ws.close()

    async def test_view_only_serves_frames_but_rejects_arm_and_drive(self):
        # Simulate even a contradictory ready sensor: this mode still cannot arm.
        self.gateway.view_only = True
        ws, state = await self.connect()
        self.assertTrue(state['view_only'])
        response = await self.client.get('/api/frame.jpg')
        self.assertEqual(response.status, 200)
        self.assertEqual(await response.read(), b'jpeg-test-bytes')
        await ws.send_json(dict(type='arm', challenge=state['challenge'], frame='frame-1'))
        state = await ws.receive_json(timeout=1)
        self.assertFalse(state['armed'])
        await ws.send_json(dict(type='drive', challenge=state['challenge'], frame='frame-1',
                                seq=1, linear=1.0, angular=1.0))
        await asyncio.sleep(0.07)
        self.assertFalse(self.gateway.control.armed)
        self.assertTrue(self.io.outputs)
        self.assertTrue(all(command == (0, 0) for command in self.io.outputs))
        self.assertEqual(self.gateway.fault(), 'Режим просмотра: управление отключено')
        await ws.send_json({'type': 'stop'})
        await asyncio.sleep(.03)
        self.assertEqual(self.io.emergency_stops, 0)
        await ws.close()

    async def test_logout_revokes_socket_and_control(self):
        ws, state = await self.connect()
        await self.arm(ws, state)
        response = await self.client.post('/api/logout', headers={'Origin': self.origin})
        self.assertEqual(response.status, 200)
        self.assertFalse(self.gateway.control.armed)
        self.assertEqual((await self.client.get('/api/status')).status, 401)
        await ws.close()


if __name__ == '__main__':
    unittest.main()
