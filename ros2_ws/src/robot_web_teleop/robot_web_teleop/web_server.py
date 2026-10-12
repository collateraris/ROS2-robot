"""Same-origin phone UI. The io object provides ROS-independent sensor/drive I/O."""
import asyncio
import contextlib
import json
import secrets
import time
from pathlib import Path

from aiohttp import WSMsgType, web
from .control import Control


class RobotServer:
    def __init__(self, io, page, password, *, view_only=False, camera_required=True):
        if len(password) < 8:
            raise ValueError('Set ROBOT_PASSWORD to at least 8 characters')
        self.io, self.page = io, Path(page)
        self.view_only = bool(view_only)
        self.camera_required = bool(camera_required)
        self.password = password.encode('utf-8')
        self.control = Control()
        self.sessions, self.clients, self.attempts = {}, {}, {}
        self.app = web.Application(client_max_size=4096)
        self.app.add_routes([
            web.get('/', self.index), web.post('/api/login', self.login),
            web.post('/api/logout', self.logout), web.get('/api/status', self.status),
            web.get('/api/frame.jpg', self.frame), web.get('/ws', self.websocket)])
        self.app.on_startup.append(self.startup)
        self.app.on_shutdown.append(self.shutdown)

    def session(self, request):
        token = request.cookies.get('robot_session', '')
        if self.sessions.get(token, 0) <= time.monotonic():
            raise web.HTTPUnauthorized(text='Войди с паролем робота')
        return token

    @staticmethod
    def origin(request):
        if request.headers.get('Origin') != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='Запрос должен идти со страницы робота')

    async def index(self, request):
        return web.FileResponse(self.page, headers={
            'Cache-Control': 'no-store', 'X-Frame-Options': 'DENY'})

    async def login(self, request):
        self.origin(request)
        now, peer = time.monotonic(), request.remote
        self.attempts = {k: v for k, v in self.attempts.items() if now - v[0] < 60}
        started, count = self.attempts.get(peer, (now, 0))
        if count >= 8:
            raise web.HTTPTooManyRequests(text='Подожди минуту перед новой попыткой')
        self.attempts[peer] = (started, count + 1)
        try:
            data = await request.json()
            supplied = data.get('password', '') if isinstance(data, dict) else ''
        except (ValueError, UnicodeError):
            raise web.HTTPBadRequest(text='Ожидается JSON')
        if not isinstance(supplied, str) or not secrets.compare_digest(
                supplied.encode('utf-8'), self.password):
            raise web.HTTPUnauthorized(text='Неверный пароль')
        self.sessions = {k: v for k, v in self.sessions.items() if v > now}
        if len(self.sessions) >= 16:
            raise web.HTTPServiceUnavailable(text='Слишком много активных сессий')
        token = secrets.token_urlsafe(32)
        self.sessions[token] = now + 8 * 3600
        self.attempts.pop(peer, None)
        response = web.json_response({'authenticated': True})
        response.set_cookie('robot_session', token, httponly=True,
                            samesite='Strict', max_age=8 * 3600, path='/')
        response.headers['Cache-Control'] = 'no-store'
        return response

    async def logout(self, request):
        self.origin(request)
        token = self.session(request)
        self.sessions.pop(token, None)
        for cid, (ws, session) in tuple(self.clients.items()):
            if session == token:
                self.disconnect(cid)
                await ws.close()
        response = web.json_response({'authenticated': False})
        response.del_cookie('robot_session', path='/')
        return response

    def state(self, client=None):
        health = self.io.health()
        return dict(type='state', authenticated=True, armed=self.control.armed,
                    view_only=self.view_only,
                    camera_required=self.camera_required,
                    owner=self.control.owner == client and client is not None,
                    reason=self.control.reason, **health)

    async def status(self, request):
        self.session(request)
        return web.json_response(self.state(), headers={'Cache-Control': 'no-store'})

    async def frame(self, request):
        self.session(request)
        data, token = self.io.frame()
        if data is None:
            raise web.HTTPServiceUnavailable(text='Нет свежего кадра')
        return web.Response(body=data, content_type='image/jpeg', headers={
            'Cache-Control': 'no-store', 'X-Frame-Token': token})

    def fault(self, frame_token=None, check_frame=False):
        if self.view_only:
            return 'Режим просмотра: управление отключено'
        health = self.io.health()
        if health.get('drive_source') == 'bluetooth':
            return 'Управляет Bluetooth. Отпустите кнопку Bluetooth для возврата к Wi-Fi'
        if not health['drive_ready']:
            return 'Arduino не готова'
        if self.camera_required and not health['camera_fresh']:
            return 'Камера не передаёт свежие кадры'
        if self.camera_required and check_frame and not self.io.frame_fresh(frame_token):
            return 'Телефон не подтвердил свежий кадр'
        return ''

    def disconnect(self, client):
        if self.control.owner == client:
            self.control.stop('Телефон отключился')
            self.io.publish(0.0, 0.0)

    async def websocket(self, request):
        self.origin(request)
        session = self.session(request)
        if len(self.clients) >= 8:
            raise web.HTTPServiceUnavailable(text='Слишком много подключений')
        ws = web.WebSocketResponse(heartbeat=5, timeout=0.3, max_msg_size=2048)
        await ws.prepare(request)
        client = secrets.token_urlsafe(12)
        self.clients[client] = (ws, session)
        sender = asyncio.create_task(self.send_states(client, ws, session))
        try:
            async for packet in ws:
                if self.sessions.get(session, 0) <= time.monotonic():
                    break
                if packet.type != WSMsgType.TEXT:
                    break
                try:
                    message = json.loads(packet.data)
                    if not isinstance(message, dict):
                        raise ValueError('object required')
                    kind = message.get('type')
                    if kind == 'stop':
                        self.control.stop()
                        self.io.publish(0.0, 0.0)
                        if not self.view_only:
                            self.io.emergency_stop()
                    elif kind == 'release':
                        # Browser focus/lease loss releases only its own Wi-Fi
                        # control. In particular, a BT takeover must not send E.
                        self.disconnect(client)
                    elif kind in ('arm', 'drive'):
                        # Another viewer's invalid frame must not stop the owner.
                        if self.control.owner not in (None, client):
                            continue
                        fault = self.fault(message.get('frame'), check_frame=True)
                        method = getattr(self.control, kind)
                        method(client, message, time.monotonic(), fault)
                    else:
                        raise ValueError('unknown message')
                except (ValueError, TypeError, OverflowError):
                    self.disconnect(client)
                    break
        finally:
            self.disconnect(client)
            self.clients.pop(client, None)
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await sender
            await ws.close()
        return ws

    async def send_states(self, client, ws, session):
        try:
            while self.sessions.get(session, 0) > time.monotonic():
                state = self.state(client)
                state['challenge'] = self.control.issue(client, time.monotonic())
                await asyncio.wait_for(ws.send_json(state), timeout=0.2)
                await asyncio.sleep(0.1)
        except (ConnectionError, asyncio.TimeoutError, RuntimeError):
            pass
        finally:
            self.disconnect(client)
            await ws.close()

    async def ticker(self):
        try:
            while True:
                linear, angular = self.control.tick(time.monotonic(), self.fault())
                self.io.publish(linear, angular)
                await asyncio.sleep(0.05)
        finally:
            self.control.stop('Сервер остановлен')
            self.io.publish(0.0, 0.0)

    async def startup(self, app):
        self.ticker_task = asyncio.create_task(self.ticker())

    async def shutdown(self, app):
        self.ticker_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self.ticker_task
        for ws, _ in tuple(self.clients.values()):
            await ws.close(code=1001, message=b'shutdown')
