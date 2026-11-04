#!/usr/bin/env python3
import sys
import time

import rclpy
from as2_python_api.drone_interface import DroneInterface

# CONFIGURATION
DRONE_NAMES = [
    "drone0",
    "drone1",
    "drone2",
    "drone3",
    "drone4",
]

LEADER = "drone0"

TAKEOFF_HEIGHT = 1.0
TAKEOFF_SPEED = 0.5
MISSION_SPEED = 0.6
LAND_SPEED = 0.4
FRAME_ID = "earth"
STABILIZATION_TIME = 3.0
FORMATION_SPACING = 1.5

# Leader's nominal formation reference position.
LEADER_X = 0.0
LEADER_Y = 0.0
FLIGHT_HEIGHT = TAKEOFF_HEIGHT

# FORMATION OFFSETS
# Positions are relative to the leader's nominal
# reference position, using the earth frame.

# LINE: followers form a line behind the leader.
LINE_OFFSETS = {
    "drone1": (-FORMATION_SPACING, 0.0),
    "drone2": (-2 * FORMATION_SPACING, 0.0),
    "drone3": (-3 * FORMATION_SPACING, 0.0),
    "drone4": (-4 * FORMATION_SPACING, 0.0),
}

# V formation: two drones on each side.
V_OFFSETS = {
    "drone1": (-FORMATION_SPACING, FORMATION_SPACING),
    "drone2": (-FORMATION_SPACING, -FORMATION_SPACING),
    "drone3": (-2 * FORMATION_SPACING, 2 * FORMATION_SPACING),
    "drone4": (-2 * FORMATION_SPACING, -2 * FORMATION_SPACING),
}


# DRONE OPERATIONS
def initialize_drones():
    """Create the five AeroStack2 drone interfaces."""
    drones = {}

    for name in DRONE_NAMES:
        print(f"[INIT] Connecting to {name}...")
        drones[name] = DroneInterface(
            name,
            verbose=True,
            use_sim_time=True,
        )

    print("[INIT] All five interfaces initialized.")
    return drones


def takeoff_all(drones):
    """Arm, enter offboard mode, and take off."""
    print("\n[TAKEOFF] Preparing all five drones.")

    # Arm all drones first.
    for name in DRONE_NAMES:
        print(f"[ARM] {name}")
        drones[name].arm()

    # Enter offboard mode.
    for name in DRONE_NAMES:
        print(f"[OFFBOARD] {name}")
        drones[name].offboard()

    # Take off one at a time, waiting for each
    # blocking takeoff operation to complete.
    for name in DRONE_NAMES:
        print(f"[TAKEOFF] {name}")
        drones[name].takeoff(
            TAKEOFF_HEIGHT,
            TAKEOFF_SPEED,
            True,
        )

    print(f"[TAKEOFF] Waiting {STABILIZATION_TIME} seconds.")
    time.sleep(STABILIZATION_TIME)
    print("[TAKEOFF] Sequence completed.")


def move_to_formation(drones, formation_name):
    if formation_name == "LINE":
        offsets = LINE_OFFSETS
    elif formation_name == "V_FORMATION":
        offsets = V_OFFSETS
    else:
        raise ValueError(f"Unknown formation: {formation_name}")

    print(f"\n[LEADER COMMAND] {LEADER} selected {formation_name}")

    # Hold the leader at its nominal reference point.
    print("[LEADER] Moving to reference position.")
    drones[LEADER].go_to.go_to(
        LEADER_X,
        LEADER_Y,
        FLIGHT_HEIGHT,
        MISSION_SPEED,
        FRAME_ID,
    )

    # Command each follower to its formation position.
    for name in DRONE_NAMES:
        if name == LEADER:
            continue

        offset_x, offset_y = offsets[name]
        target_x = LEADER_X + offset_x
        target_y = LEADER_Y + offset_y

        print(
            f"[FOLLOWER] {name} -> "
            f"x={target_x:.2f}, "
            f"y={target_y:.2f}, "
            f"z={FLIGHT_HEIGHT:.2f}"
        )

        drones[name].go_to.go_to(
            target_x,
            target_y,
            FLIGHT_HEIGHT,
            MISSION_SPEED,
            FRAME_ID,
        )

    print(f"[FORMATION] {formation_name} command completed.")


def land_all(drones):
    """Request landing for all five drones."""
    print("\n[LAND] Starting landing sequence.")

    for name in DRONE_NAMES:
        print(f"[LAND] {name}")
        drones[name].land(LAND_SPEED, True)

    print("[LAND] Landing sequence completed.")


# COMMAND LOOP
def main():
    rclpy.init()

    drones = {}
    airborne = False

    print("=" * 55)
    print(" FIVE-DRONE LEADER-FOLLOWER FORMATION DEMO")
    print("=" * 55)
    print(f"Leader:    {LEADER}")
    print(f"Followers: {', '.join(DRONE_NAMES[1:])}")
    print()

    try:
        drones = initialize_drones()

        print("\nAvailable commands:")
        print("  TAKEOFF      - Take off all five drones")
        print("  LINE         - Switch to line formation")
        print("  V_FORMATION  - Switch to V formation")
        print("  LAND         - Land all five drones")
        print("  EXIT         - Exit the demo")
        print()

        while True:
            command = input("Leader command > ").strip().upper()

            if command == "TAKEOFF":
                if airborne:
                    print("[INFO] Drones are already marked airborne.")
                    continue
                takeoff_all(drones)
                airborne = True

            elif command in ("LINE", "V_FORMATION"):
                if not airborne:
                    print("[WARNING] Enter TAKEOFF first.")
                    continue
                move_to_formation(drones, command)

            elif command == "LAND":
                if not airborne:
                    print("[INFO] The takeoff sequence has not run.")
                    continue
                land_all(drones)
                airborne = False

            elif command == "EXIT":
                if airborne:
                    print(
                        "[SAFETY] Drones are marked airborne. "
                        "Use LAND before EXIT."
                    )
                    continue
                break

            elif command == "":
                continue

            else:
                print("[ERROR] Unknown command.")

    except KeyboardInterrupt:
        print(
            "\n[INTERRUPTED] Check the drones' actual state "
            "in Gazebo and land safely if necessary."
        )

    except Exception as exc:
        print(f"\n[ERROR] Demo stopped: {exc}")
        print(
            "[SAFETY] Check the actual state of every drone "
            "before issuing further commands."
        )
        raise

    finally:
        for name, drone in drones.items():
            try:
                drone.shutdown()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
