# xbox_mapping_test.py

import pygame
import time

def main():
    pygame.init()
    pygame.joystick.init()

    print("Waiting for joystick...")
    while pygame.joystick.get_count() == 0:
        time.sleep(0.5)
        pygame.joystick.quit()
        pygame.joystick.init()
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Joystick initialized: {js.get_name()}")
    print(f"  Number of buttons: {js.get_numbuttons()}")
    print(f"  Number of axes   : {js.get_numaxes()}")
    print("-" * 40)
    print("Press buttons or move sticks/triggers to see mapping.")
    print("Ctrl+C to exit.")
    print()

    try:
        while True:
            pygame.event.pump()  # ??? ? ????

            # ?? ???
            pressed = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            if pressed:
                print("Pressed buttons:", pressed)

            # ?? ? ?
            axes = [round(js.get_axis(i), 3) for i in range(js.get_numaxes())]
            print("Axes values     :", axes)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        js.quit()
        pygame.quit()

if __name__ == "__main__":
    main()
