import Robot
import numpy as np

# check installation
print("Robot module loaded from:", Robot.__file__)
print("RPC class:", Robot.RPC)

# connect to robot
robot = Robot.RPC('192.168.58.2')

# read current joint angles of robot
currPos = robot.robot_state_pkg.jt_cur_pos

# move robot to new position (e.g., rotate first joint by 15 degrees)
newPos = np.array(currPos)
newPos[0] += 15
rtn = robot.MoveJ(joint_pos=newPos, tool=0, user=0)

# close robot connection
robot.CloseRPC()
