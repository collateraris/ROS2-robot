"""Single-controller lease and dead-man switch; independent of ROS and HTTP."""
import math
import secrets


class Control:
    def __init__(self, timeout=0.25, challenge_timeout=0.30):
        self.timeout = timeout
        self.challenge_timeout = challenge_timeout
        self.owner = None
        self.armed = False
        self.linear = self.angular = 0.0
        self.last_drive = float('-inf')
        self.last_seq = -1
        self.reason = 'Нажми «Взять управление»'
        self.challenges = {}

    def issue(self, client, now):
        self.challenges = {k: v for k, v in self.challenges.items()
                           if 0 <= now - v[1] <= self.challenge_timeout}
        token = secrets.token_urlsafe(12)
        self.challenges[token] = (client, now)
        return token

    def fresh(self, client, token, now):
        item = self.challenges.get(token) if isinstance(token, str) else None
        return bool(item and item[0] == client and
                    0 <= now - item[1] <= self.challenge_timeout)

    def stop(self, reason='Остановлено. Включи управление снова.'):
        self.owner = None
        self.armed = False
        self.linear = self.angular = 0.0
        self.last_seq = -1
        self.reason = reason
        self.challenges.clear()  # Queued old arm messages cannot re-arm.

    def tick(self, now, fault=''):
        if self.armed and (fault or now - self.last_drive > self.timeout):
            self.stop(fault or 'Нет свежих команд с телефона')
        return self.linear, self.angular

    def arm(self, client, message, now, fault=''):
        if self.owner not in (None, client):
            return False
        was_armed = self.armed
        self.tick(now, fault)  # Expiry invalidates challenges before any new arm.
        if was_armed:
            self.stop(fault or 'Повторное включение: отпусти пульт и включи снова')
            return False
        if fault or not self.fresh(client, message.get('challenge'), now):
            self.reason = fault or 'Задержка связи: попробуй включить снова'
            return False
        self.owner, self.armed = client, True
        self.linear = self.angular = 0.0
        self.last_drive, self.last_seq = now, -1
        self.reason = 'Удерживай джойстик для движения'
        return True

    def drive(self, client, message, now, fault=''):
        if self.owner != client:
            return False
        self.tick(now, fault)
        if not self.armed or self.owner != client:
            return False
        seq = message.get('seq')
        values = (message.get('linear'), message.get('angular'))
        valid = (type(seq) is int and 0 <= seq <= 2**53 - 1 and seq > self.last_seq
                 and all(type(x) in (int, float) and math.isfinite(x)
                         and abs(x) <= 1.0 for x in values)
                 and self.fresh(client, message.get('challenge'), now))
        if not valid:
            self.stop('Устаревшая или некорректная команда')
            return False
        self.linear, self.angular = map(float, values)
        self.last_drive, self.last_seq = now, seq
        return True
