# file: both_sticks_percent.py
from evdev import InputDevice, ecodes, list_devices
from select import select
import sys, time

# De globala variabler som ska importeras till CAN koden
MODE = "manual"   # global variabel för körläget
ly = 0.0
ry = 0.0
lt = 0.0
rt = 0.0
bucket = 0.0
    
TARGET_AXES = (ecodes.ABS_Y, ecodes.ABS_RY)

def pick_device():
    for p in list_devices():
        d = InputDevice(p)
        if "Logitech" in d.name or "Gamepad" in d.name or "Controller" in d.name:
            return d
    paths = list_devices()
    if not paths:
        raise SystemExit("No input device found.")
    return InputDevice(paths[0])

def get_range(dev, code):
    info = dev.absinfo(code)
    if info:
        return info.min, info.max
    return -32768, 32767  # common fallback

def to_percent(value, vmin, vmax):
    center = (vmin + vmax) / 2.0
    denom  = max(abs(vmax - center), abs(vmin - center)) or 1
    pct = ((value - center) / denom) * 100.0
    if abs(pct) < 5.0:
        return 0.0
    return max(-100.0, min(100.0, pct))  # clamp between -100 and 100, just return pct

def main():
    global MODE, ly, ry, lt, rt, bucket
    gamepad = pick_device()
    print(f"Reading: {gamepad.path} – {gamepad.name}")

    # ranges
    min_y,  max_y  = get_range(gamepad, ecodes.ABS_Y)
    min_ry, max_ry = get_range(gamepad, ecodes.ABS_RY)
    

    # initial line
    print(f"MODE: {MODE.upper()} | LY:  +0% | RY:  +0% | LT:  0% | RT:  0%", end="", flush=True)

    try:
        while True:
            # Wait up to 20 ms for events; None if timeout
            r, _, _ = select([gamepad], [], [], 0.02)

            if r:
                # There are events ready; read all pending ones
                for e in gamepad.read():
                    
                    # Toggle button for manual or automatic mode
                    if e.type == ecodes.EV_KEY and e.code == ecodes.BTN_WEST and e.value == 1:
                        if MODE == "manual":
                            MODE = "automatic"  
                        else:
                            MODE = "manual"
                    
                    if e.type != ecodes.EV_ABS:
                        continue
                    if e.code == ecodes.ABS_Y:
                        ly = to_percent(e.value, min_y, max_y)
                    elif e.code == ecodes.ABS_RY:
                        ry = to_percent(e.value, min_ry, max_ry)  # invert RY if desired
                        
                        
                    elif e.type == ecodes.EV_ABS and e.code == ecodes.ABS_Z:     # LT
                        lt = (e.value / 256.0) * 100.0
                        
                    elif e.type == ecodes.EV_ABS and e.code == ecodes.ABS_RZ:    # RT
                        rt = (e.value / 256.0) * 100.0
                        
                    #if rt > 5.0:
                        #bucket = rt
                    #elif lt > 5.0:
                        #bucket = -lt
                    #else:
                        #bucket = 0.0


                # update immediately after handling a batch
                #sys.stdout.write(f"\rMODE: {MODE.upper()} | LY: {ly:+.0f}% | RY: {ry:+.0f}% | LT: {lt:>3.0f}% | RT: {rt:>3.0f}%")
                #sys.stdout.flush()
            #else:
                # No events this tick — still refresh the line (keeps UI snappy)
                #sys.stdout.write(f"\rMODE: {MODE.upper()} | LY: {ly:+.0f}% | RY: {ry:+.0f}% | LT: {lt:>3.0f}% | RT: {rt:>3.0f}%")
                #sys.stdout.flush()
                # tiny sleep to keep CPU low (optional; select timeout already throttles)
                #time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting…")

if __name__ == "__main__":
    main()
