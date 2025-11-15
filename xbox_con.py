import threading
import time
import pygame

class ControllerState:
    def __init__(self, rt=0, lt=0, ls_x=0, a=False, y=False):
        # rt, lt: 0..65534
        self.rt   = rt
        self.lt   = lt
        # ls_x: -32767..32767
        self.ls_x = ls_x
        self.a    = a
        self.y    = y

# ?? ?? ??
rt = 0
lt = 0
ls_x = 0
a = False
y = False

_lock  = threading.Lock()

# Pygame ???
pygame.init()
pygame.joystick.init()

print("Waiting for joystick?")
while True:
    if pygame.joystick.get_count() > 0:
        _j = pygame.joystick.Joystick(0)
        _j.init()
        print("Joystick initialized.")
        break
    time.sleep(1)
    pygame.joystick.quit()
    pygame.joystick.init()

# ?? ?? ???
def _poll_loop():
    global rt, lt, ls_x, a, y
    while True:
        pygame.event.pump()
        with _lock:
            # LS X: -1.0 ~ 1.0 -> -32767~32767
            raw_ls_x = _j.get_axis(0)
            ls_x = int(raw_ls_x * 32767)

            # RT: axis 4, LT: axis 5 (? -1.0~1.0 -> 0~65534)
            axis_rt = _j.get_axis(4)
            axis_lt = _j.get_axis(5)
            rt = int((axis_rt + 1.0) * 32767)
            lt = int((axis_lt + 1.0) * 32767)

            # ?? A=0, Y=4
            a = bool(_j.get_button(0))
            y = bool(_j.get_button(4))

        time.sleep(0.001)

_thread = threading.Thread(target=_poll_loop, daemon=True)
_thread.start()

# ??? ?? ?? ??
def get_controller_state():
    with _lock:
        return ControllerState(rt=rt, lt=lt, ls_x=ls_x, a=a, y=y)

if __name__ == '__main__':
    # ?? ??? ??
    try:
        while True:
            state = get_controller_state()
            print(f"A={state.a}  Y={state.y}  LS_X={state.ls_x}  RT={state.rt}  LT={state.lt}")
            time.sleep(0.000001)
    except KeyboardInterrupt:
        print("Exiting...")
