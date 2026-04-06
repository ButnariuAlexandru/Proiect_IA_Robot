import time
from enum import Enum
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ---------------- PARAMETRI ----------------

V_FORWARD = 2.0
V_BACKWARD = -2.0
V_TURN = 2.0

STOP_DISTANCE = 0.5
BACKWARD_TIME = 1.0
TURN_90 = 1.57
TURN_180 = 3.14

CONTROL_DT = 0.05

FRONT_SENSORS = [2, 3, 4, 5]
LEFT_SENSORS = [0, 1, 2]
RIGHT_SENSORS = [5, 6, 7]

SENSOR_MAX = 1.0


# ---------------- MASINA DE STARI ----------------

class RobotState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURNING = 3


# ---------------- FUNCTII SENZORI ----------------

def get_min_distance(sim, sensors, indices):
    min_dist = SENSOR_MAX

    for i in indices:
        result, distance, *_ = sim.readProximitySensor(sensors[i])

        if result and distance < min_dist:
            min_dist = distance

    return min_dist


def set_velocity(sim, left_motor, right_motor, v_left, v_right):
    sim.setJointTargetVelocity(left_motor, v_left)
    sim.setJointTargetVelocity(right_motor, v_right)


# ---------------- PROGRAM PRINCIPAL ----------------

def main():

    client = RemoteAPIClient()
    sim = client.require('sim')

    left_motor = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')

    sensors = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]

    state = RobotState.FORWARD
    state_start_time = time.time()

    turn_direction = 1
    turn_time = TURN_90

    sim.startSimulation()

    try:

        while True:

            dist_front = get_min_distance(sim, sensors, FRONT_SENSORS)
            dist_left = get_min_distance(sim, sensors, LEFT_SENSORS)
            dist_right = get_min_distance(sim, sensors, RIGHT_SENSORS)

            elapsed = time.time() - state_start_time

            # ---------------- FORWARD ----------------

            if state == RobotState.FORWARD:

                set_velocity(sim, left_motor, right_motor,
                             V_FORWARD, V_FORWARD)

                if dist_front < STOP_DISTANCE:

                    state = RobotState.BACKWARD
                    state_start_time = time.time()

            # ---------------- BACKWARD ----------------

            elif state == RobotState.BACKWARD:

                set_velocity(sim, left_motor, right_motor,
                             V_BACKWARD, V_BACKWARD)

                if elapsed > BACKWARD_TIME:

                    state = RobotState.TURNING
                    state_start_time = time.time()

                    # alegem directia 
                    if dist_left > dist_right:
                        turn_direction = 1
                    else:
                        turn_direction = -1

                    # detectie colț (capăt zid)
                    if (dist_front < STOP_DISTANCE and
                        dist_left < STOP_DISTANCE and
                        dist_right < STOP_DISTANCE):

                        turn_time = TURN_180
                    else:
                        turn_time = TURN_90

            # ---------------- TURNING ----------------

            elif state == RobotState.TURNING:

                if turn_direction == 1:
                    set_velocity(sim, left_motor, right_motor,
                                 -V_TURN, V_TURN)
                else:
                    set_velocity(sim, left_motor, right_motor,
                                 V_TURN, -V_TURN)

                if elapsed > turn_time:
                    state = RobotState.FORWARD
                    state_start_time = time.time()

            print(f"F={dist_front:.2f}  L={dist_left:.2f}  R={dist_right:.2f}  STATE={state}")

            time.sleep(CONTROL_DT)

    except KeyboardInterrupt:
        pass

    finally:

        set_velocity(sim, left_motor, right_motor, 0, 0)
        sim.stopSimulation()


if __name__ == "__main__":
    main()