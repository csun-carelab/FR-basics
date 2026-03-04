import Robot
import time

# Establish a connection to the robot controller.
robot = Robot.RPC("192.168.58.2")

JP1 = [117.408, -86.777, 81.499, -87.788, -92.964, 92.959]
DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]

JP2 = [72.515, -86.774, 81.525, -87.724, -91.964, 92.958]
DP2 = [-65.169, -529.17, 518.018, -177.189, 3.119, 69.556]

DP2_h = [-65.169, -529.17, 528.018, -177.189, 3.119, 69.556]

JP3 = [89.281, -102.959, 81.527, -69.955, -86.755, 92.958]
DP3 = [102.939, -378.069, 613.165, 176.687, 1.217, 86.329]


def startjog():
    """Jog test."""
    print("Joint 1, negative direction, 90 deg, speed 30")
    error = robot.StartJOG(ref=0, nb=1, dir=0, max_dis=10, vel=30)
    print("StartJOG return", error)


def stopjog():
    """Jog deceleration stop test."""
    robot.StartJOG(ref=0, nb=1, dir=1, max_dis=90, vel=50)
    time.sleep(3)
    print("Stopping jog")
    error = robot.StopJOG(ref=1)
    print("StopJOG return", error)


def immstopjog():
    """Immediate jog stop test."""
    robot.StartJOG(ref=0, nb=1, dir=1, max_dis=90, vel=50)
    time.sleep(3)
    print("Immediate jog stop")
    error = robot.ImmStopJOG()
    print("ImmStopJOG return", error)


def movej():
    """MoveJ test."""
    jp = [28.166, -108.269, -59.859, -87, 94.532, -0.7]
    error = robot.MoveJ(joint_pos=jp, tool=0, user=0, vel=30)
    print("MoveJ return", error)


def movel():
    """MoveL test."""
    error = robot.MoveL(desc_pos=DP1, tool=0, user=0, vel=30)
    print("MoveL return", error)


def movecart():
    """MoveCart test."""
    error = robot.MoveCart(desc_pos=DP2, tool=0, user=0, vel=30)
    print("MoveCart return", error)


def movec():
    """MoveC test."""
    robot.MoveCart(desc_pos=DP2, tool=0, user=0, vel=30)
    error = robot.MoveC(desc_pos_p=DP3, tool_p=0, user_p=0, desc_pos_t=DP1, tool_t=0, user_t=0)
    print("MoveC return", error)


def circle():
    """Circle test."""
    error = robot.Circle(desc_pos_p=DP3, tool_p=0, user_p=0, desc_pos_t=DP2, tool_t=0, user_t=0)
    print("Circle return", error)


def newspiral():
    """NewSpiral test."""
    error = robot.NewSpiral(desc_pos=DP2_h, tool=0, user=0, param=[5.0, 10, 30, 10, 5, 0])
    print("NewSpiral return", error)


def servoj():
    """ServoJ test."""
    error, pos = robot.GetActualJointPosDegree()
    robot.ServoMoveStart()
    i = 0
    while i < 100:
        time.sleep(0.1)
        pos[4] -= 0.2
        error = robot.ServoJ(joint_pos=pos, axisPos=[0, 0, 0, 0])
        i += 1
    robot.ServoMoveEnd()
    print("ServoJ return", error)


def servocart():
    """ServoCart test."""
    robot.ServoMoveStart()
    pos = [0, 0, 0.2, 0, 0, 0]
    i = 0
    while i < 200:
        time.sleep(0.008)
        error = robot.ServoCart(mode=1, desc_pos=pos)
        i += 1
    robot.ServoMoveEnd()
    print("ServoCart return", error)


def splineptp():
    """SplinePTP test."""
    robot.SplineStart()
    error = robot.SplinePTP(joint_pos=JP2, tool=0, user=0)
    robot.SplineEnd()
    print("SplinePTP return", error)


def newsplineptp():
    """NewSplinePTP test."""
    robot.NewSplineStart(type=0)
    pos1 = [-104.846, 309.573, 336.647, 179.681, -0.419, -92.692]
    pos2 = [-194.846, 309.573, 336.647, 179.681, -0.419, -92.692]
    pos3 = [-254.846, 259.573, 336.647, 179.681, -0.419, -92.692]
    pos4 = [-304.846, 259.573, 336.647, 179.681, -0.419, -92.692]
    robot.MoveCart(desc_pos=pos1, tool=0, user=0, vel=30)
    robot.NewSplinePoint(desc_pos=pos1, tool=0, user=0, lastFlag=0)
    robot.NewSplinePoint(desc_pos=pos2, tool=0, user=0, lastFlag=0)
    robot.NewSplinePoint(desc_pos=pos3, tool=0, user=0, lastFlag=0)
    error = robot.NewSplinePoint(desc_pos=pos4, tool=0, user=0, lastFlag=1)
    robot.NewSplineEnd()
    print("NewSplinePTP return", error)


def pointsoffset():
    """PointsOffset test."""
    robot.PointsOffsetEnable(flag=0, offset_pos=[0, 0, -100, 0, 0, 0])
    error = robot.MoveL(desc_pos=DP1, tool=0, user=0)
    robot.PointsOffsetDisable()
    print("PointsOffset return", error)


def jointoverspeedprotect():
    """Overspeed protection test."""
    error = robot.MoveL(desc_pos=DP1, tool=0, vel=100, user=0, overSpeedStrategy=3, speedPercent=100)
    print("Overspeed protection return", error)


def movej_test():
    """MoveJ test."""
    jp1 = [130.124, -99.15, -110.123, -62.577, 90.997, -81.748]
    error = robot.MoveJ(joint_pos=jp1, tool=0, user=0, vel=30)
    print("MoveJ return", error)


# movej_test()
# startjog()
# stopjog()
# immstopjog()
# movej()
# movel()
# movecart()
# movec()
# circle()
# newspiral()
# servoj()  # Disabled by default: potentially dangerous auto-run
# servocart()
# splineptp()
# newsplineptp()
# pointsoffset()
# jointoverspeedprotect()
