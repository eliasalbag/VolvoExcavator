import threading
import time
from inputs import devices


# Smoothing factor
ALPHA = 0.002


gamepad = devices.gamepads[0]

_state = {
    "ly": 0,
    "rx": 0,
    "tl": 0,
    "rl": 0,
    "ly_sm": 0,
    "rx_sm": 0,
    "tl_sm": 0,
    "rl_sm": 0
}


def _reader():
    while True:
        try:
            events = gamepad.read()
            for e in events:
                if e.code == "ABS_Y": _state["ly"] = e.state
                elif e.code == "ABS_RX": _state["rx"] = e.state
                elif e.code == "BTN_TL": _state["tl"] = e.state
                elif e.code == "BTN_TR": _state["rl"] = e.state
        except Exception:
            pass
        time.sleep(0.005)


threading.Thread(target=_reader, daemon=True).start()


def normalize_deadspace_smoothing(x, x_sm):
    if x < 0:
        x = round((x - 128) / 328.96)
    else:
        x = round((x - 128) / 326.39)
    if abs(x) < 8:
        return 0, 0
    else:
        # Apply smoothing
        x_sm = x_sm + ALPHA * (x - x_sm)
        return x, x_sm


def get_controller_values():
    ly, _state["ly_sm"] = normalize_deadspace_smoothing(_state["ly"], _state["ly_sm"])
    rx, _state["rx_sm"] = normalize_deadspace_smoothing(_state["rx"], _state["rx_sm"])
    if _state["tl"] != 0:
        tl = 100
    else:
        tl = 0
    if _state["rl"] != 0:
        tr = -100
    else:
        tr = 0
    bucket_tilt = tr + tl
    return ly, rx, bucket_tilt
