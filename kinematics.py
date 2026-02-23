import Robot

# connect to robot
robot = Robot.RPC('192.168.58.2')

# example joint angles
j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]

# example cartesian position
desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]

# inverse kinematics
error, inverseRtn = robot.GetInverseKin(0, desc_pos=desc_pos1, config=-1)
print(f"dcs1 GetInverseKin rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
      f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")


error, inverseRtn = robot.GetInverseKinRef(0, desc_pos=desc_pos1, joint_pos_ref=j1)
print(f"dcs1 GetInverseKinRef rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
      f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")
error, hasResult = robot.GetInverseKinHasSolution(0, desc_pos=desc_pos1, joint_pos_ref=j1)
print(f"dcs1 GetInverseKinRef result {hasResult}")
error, forwordResult = robot.GetForwardKin(j1)
print(f"jpos1 forwordResult rtn is {forwordResult[0]}, {forwordResult[1]}, {forwordResult[2]}, "
      f"{forwordResult[3]}, {forwordResult[4]}, {forwordResult[5]}")

robot.CloseRPC()