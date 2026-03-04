import Robot
import time

# A connection is established with the robot controller.
robot = Robot.RPC("192.168.58.2")

JP1 = [-68.030, -63.537, -105.223, -78.368, 72.828, 24.876]
DP1 = [-60.984, -533.958, 279.089, -22.052, -4.777, 172.406]

JP2 = [-80.916, -76.030, -108.901, -70.956, 99.026, -74.533]
DP2 = [36.750, -488.721, 145.781, -37.539, -11.211, -96.491]

JP3 = [-86.898, -95.200, -103.665, -70.570, 98.266, -93.321]
DP3 = [-21.462, -509.234, 25.706, -41.780, -1.042, -83.611]

JP4 = [-85.364, -102.697, -94.674, -70.557, 95.302, -93.116]
DP4 = [-24.075, -580.525, 25.881, -44.818, -2.357, -82.259]

JP5 = [-78.815, -94.279, -105.315, -65.348, 87.328, 3.220]
DP5 = [-29.155, -580.477, 25.884, -44.795, -2.374, -172.261]

JP6 = [-81.057, -94.494, -105.107, -65.241, 87.527, 0.987]
DP6 = [-49.270, -580.460, 25.886, -44.796, -2.374, -172.263]

JP7 = [-76.519, -101.428, -94.915, -76.521, 85.041, 95.758]
DP7 = [-54.189, -580.362, 25.878, -44.779, -2.353, 97.740]

JP8 = [-74.406, -90.991, -106.574, -75.480, 85.150, 97.875]
DP8 = [-54.142, -503.358, 25.865, -44.780, -2.353, 97.740]

# WARNING: tool=7 is setup-specific. Change to match your WebApp tool config.
tool = 7
user = 0
vel = 60.0


def wait_robot_motion_done(robot, timeout_s=5.0, poll_s=0.1):
    max_checks = max(1, int(timeout_s / poll_s))
    for _ in range(max_checks):
        time.sleep(poll_s)
        ret = robot.GetRobotMotionDone()
        if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0 and ret[1] == 1:
            return True
    return False


def main():
    time.sleep(5)
    error = robot.MoveJ(joint_pos=JP1, tool=tool, user=user, desc_pos=DP1, vel=vel)
    print("MoveJ return ", error)
    done = wait_robot_motion_done(robot)
    print("GetRobotMotionDone after JP1 ", done)

    error = robot.MoveJ(joint_pos=JP2, tool=tool, user=user, desc_pos=DP2, vel=vel)
    print("MoveJ return ", error)
    done = wait_robot_motion_done(robot)
    print("GetRobotMotionDone after JP2 ", done)

    vel_l = 10.0
    error = robot.MoveL(DP3, tool, user, joint_pos=JP3, vel=vel_l)
    print("MoveL return ", error)

    error = robot.SetOaccScale(100)
    print("SetOaccScale return ", error)

    error = robot.MoveL(DP4, tool, user, joint_pos=JP4, vel=vel_l)
    print("MoveL return ", error)

    error = robot.AngularSpeedStart(50)
    print("AngularSpeedStart return ", error)

    error = robot.MoveL(DP5, tool, user, joint_pos=JP5, vel=vel_l)
    print("MoveL return ", error)

    error = robot.AngularSpeedEnd()
    print("AngularSpeedEnd return ", error)

    error = robot.MoveL(DP6, tool, user, joint_pos=JP6, vel=vel_l)
    print("MoveL return ", error)

    error = robot.AngularSpeedStart(50)
    print("AngularSpeedStart return ", error)

    error = robot.MoveL(DP7, tool, user, joint_pos=JP7, vel=vel_l)
    print("MoveL return ", error)

    error = robot.AngularSpeedEnd()
    print("AngularSpeedEnd return ", error)

    error = robot.MoveL(DP8, tool, user, joint_pos=JP8, vel=vel_l)
    print("MoveL return ", error)


if __name__ == "__main__":
    main()
