import argparse

from servo_control import set_angle  # ? ??? ? ?????

if __name__ == "__main__":
    # --angle ?? ??
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, required=True, help="Steering angle")
    args = parser.parse_args()

    # ?? angle ??
    print(f"Received angle: {args.angle}")

    # ?? ??? ?? ??? ??
    set_angle(args.angle)

