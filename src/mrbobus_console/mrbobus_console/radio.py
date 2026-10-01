"""Console adapter for the independent UART/STOP service. No CAN access."""
import json
import threading
import time
from pathlib import Path

RADIO_OWNER = 'ELRS-radio'


def read_radio():
    try:
        data = json.loads(Path('/run/mrbobus-stop/radio.json').read_text())
        if not 0 <= time.monotonic()-data['at'] < .3:
            return None
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return None


class RadioBridge:
    def __init__(self, node):
        self.node = node
        self.error = ''
        self.arm_thread = None
        threading.Thread(target=self.run, daemon=True).start()

    def arm(self):
        try:
            self.node.arm(RADIO_OWNER, radio=True)
            self.error = ''
        except Exception as e:
            self.error = str(e)

    def run(self):
        previous = None
        sequence = 0
        while self.node.running:
            time.sleep(.04)
            data = read_radio()
            try:
                if data is None:
                    if self.node.gate.active or self.node.gate.owner:
                        self.node.stop()
                    previous = None
                    continue
                tokens = (data['stop_token'], data['arm_token'])
                if previous is None:
                    previous = tokens
                    continue  # Never consume an old arm event after a restart.
                changed_stop = tokens[0] != previous[0]
                changed_arm = tokens[1] != previous[1]
                previous = tokens
                if changed_stop:
                    self.node.stop()
                if changed_arm and data['mode'] == 'manual' and data['connected'] and not data['held']:
                    if self.arm_thread is None or not self.arm_thread.is_alive():
                        self.arm_thread = threading.Thread(target=self.arm, daemon=True)
                        self.arm_thread.start()
                if data['mode'] == 'manual' and self.node.gate.active:
                    if not data['connected'] or data['held'] or not data['gear']:
                        self.node.stop()
                        continue
                    sequence += 1
                    self.node.command(dict(client=RADIO_OWNER, seq=sequence,
                        v=.50*data['throttle']*data['gear'], w=-.6*data['steering']), radio=True)
            except Exception as e:
                self.error = str(e)
                try: self.node.stop()
                except Exception: pass


def allow_source(radio=False):
    data = read_radio()
    if data is None or data['held']:
        return False
    if radio:
        return data['mode'] == 'manual' and data['connected'] and data['gear'] != 0
    return data['mode'] == 'auto'
