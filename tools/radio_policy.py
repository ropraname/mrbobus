"""Measured CRSF controls and source arbitration; no I/O or motor commands."""
class RadioPolicy:
    timeout = .3

    def __init__(self):
        self.mode = 'auto'
        self.gear = 0
        self.held = False
        self.last_rc = self.last_link = -1e9
        self.lq = 0
        self.throttle = self.steering = 0.
        self.arm_token = 0
        self.stop_token = 0
        self.connected = False
        self.channels = None
        self.reason = 'Пульт не подключён; доступен AUTO'

    def link(self, quality, now):
        self.lq = quality
        self.last_link = now

    def fresh(self, now):
        return now-self.last_rc < self.timeout and now-self.last_link < 1. and self.lq > 0

    def update(self, ch, now):
        if len(ch) != 16:
            return
        self.channels = list(ch)
        # Reject invalid/sentinel controls rather than treating them as switches.
        if any(not 150 <= ch[i] <= 1850 for i in (0, 2, 4, 5, 7)):
            return
        previous = (self.mode, self.gear, self.held)
        was_connected = self.connected and self.fresh(now)
        self.last_rc = now
        if now-self.last_link >= 1. or self.lq <= 0:
            return
        self.mode = 'manual' if ch[4] > 1400 else 'auto' if ch[4] < 600 else self.mode
        self.gear = 1 if ch[5] < 600 else -1 if ch[5] > 1400 else 0
        self.held = ch[7] > 1400
        self.throttle = max(0., min(1., (ch[2]-255)/1556))
        steering = (ch[0]-992)/819
        self.steering = 0. if abs(steering) < .06 else max(-1., min(1., steering))
        changed = self.mode != previous[0] or self.held != previous[2] or (
            self.mode == 'manual' and self.gear != previous[1])
        if changed or (not was_connected and self.mode == 'manual'):
            # Every mode/gear change first removes the previous command source.
            self.stop_token += 1
            self.reason = 'Переключение пульта: STOP'
            intentional = was_connected and (previous[2] and not self.held or
                                              previous[1] != self.gear)
            if intentional and self.mode == 'manual' and self.gear and not self.held and self.throttle == 0 and self.steering == 0:
                self.arm_token += 1
        self.connected = True
        if self.held: self.reason = 'STOP на пульте'
        elif self.mode == 'manual' and not self.gear: self.reason = 'Нейтраль: колёса свободны'

    def tick(self, now):
        if self.connected and not self.fresh(now):
            self.connected = False
            was_manual = self.mode == 'manual'
            self.mode = 'auto'
            self.held = False
            self.gear = 0
            self.throttle = self.steering = 0.
            if was_manual:
                self.stop_token += 1
            self.reason = 'Пульт отключён: AUTO; запрет пульта снят'

    def may_arm(self, now):
        if self.held:
            return False
        return self.mode == 'auto' or (self.fresh(now) and self.gear != 0 and self.throttle == 0 and self.steering == 0)

    def status(self, now):
        self.tick(now)
        return dict(mode=self.mode, gear=self.gear, held=self.held,
                    connected=self.fresh(now), throttle=self.throttle,
                    steering=self.steering, arm_token=self.arm_token,
                    stop_token=self.stop_token, reason=self.reason,
                    lq=self.lq, at=now, channels=self.channels)
