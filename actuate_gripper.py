import Robot
import time

# connect to robot
robot = Robot.RPC('192.168.58.2')

# set gripper make (4 - DH) and type (0 - parallel)
robot.SetGripperConfig(4, 0)

# activate gripper #1
robot.ActGripper(1, 1)

# small delay to wait for gripper to activate
time.sleep(1)

# close gripper
robot.MoveGripper(1, 25, 50, 50, 5000, 0, 0, 0, 0, 0)
while not robot.GetGripperMotionDone()[1][1]:
    print("Gripper moving")

# open gripper
robot.MoveGripper(1, 100, 50, 50, 5000, 0, 0, 0, 0, 0)
while not robot.GetGripperMotionDone()[1][1]:
    print("Gripper moving")
