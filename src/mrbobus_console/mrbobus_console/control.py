"""Pure command policy; no ROS, CAN, or persistent browser heartbeat."""
import math
import threading

class MotionGate:
    def __init__(self):
        self.lock=threading.RLock(); self.owner=None; self.active=False; self.epoch=0; self.last_seq=-1
    def begin_arm(self, owner):
        if not isinstance(owner,str) or not 8<=len(owner)<=80:raise ValueError('Некорректный идентификатор пульта')
        with self.lock:
            if self.owner not in (None,owner):raise ValueError('Привод занят другим пультом. Сначала STOP.')
            self.epoch+=1; self.owner=owner; self.active=False; self.last_seq=-1
            return self.epoch
    def finish_arm(self, epoch):
        with self.lock:
            if epoch!=self.epoch:raise ValueError('Включение отменено кнопкой STOP')
            self.active=True
    def check_epoch(self, epoch):
        with self.lock:
            if epoch!=self.epoch:raise ValueError('Включение отменено')
    def stop(self):
        with self.lock:self.epoch+=1;self.active=False;self.owner=None
    def command(self, owner, v, w, seq):
        with self.lock:
            if not self.active or owner!=self.owner:raise ValueError('Сначала включи привод в этом пульте')
            if isinstance(seq,bool) or not isinstance(seq,int) or seq<=self.last_seq:raise ValueError('Устаревшая команда')
            if isinstance(v,bool) or isinstance(w,bool):raise ValueError('Некорректная скорость')
            v=float(v);w=float(w)
            if not math.isfinite(v) or not math.isfinite(w):raise ValueError('Некорректная скорость')
            v=max(-.50,min(.50,v));w=max(-.6,min(.6,w))
            # Match the hardware wheel-command cap even with simultaneous turn/drive.
            demand=abs(v)+abs(w)*.29174/2
            scale=min(1.,(.045*14.)/max(demand,1e-9))
            self.last_seq=seq
            return v*scale,w*scale
