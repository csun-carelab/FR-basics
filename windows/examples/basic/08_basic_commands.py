import Robot
import time

# Establish a connection to the robot controller.
robot = Robot.RPC("192.168.58.2")


def rpc():
    """RPC communication connectivity test."""
    if robot.is_conect:
        print("Connection successful")
    else:
        print("Connection failed")


def getsdkversion():
    """Get SDK version."""
    error, sdk = robot.GetSDKVersion()
    print("SDK version: %s, %s" % (sdk[0], sdk[1]))


def getcontrollerip():
    """Get controller IP."""
    error, ip_addr = robot.GetControllerIP()
    print("Controller IP: %s" % ip_addr)


def dragteachswitch():
    """Enter and exit drag-teach mode."""
    print("Entering drag-teach mode")
    robot.DragTeachSwitch(state=1)
    time.sleep(3)
    print("Exiting drag-teach mode")
    robot.DragTeachSwitch(state=0)


def isindragteach():
    """Query whether drag-teach mode is active."""
    error, is_drag = robot.IsInDragTeach()
    if is_drag == 0:
        print("Robot is not in drag-teach mode")
    elif is_drag == 1:
        print("Robot is in drag-teach mode")


def robotenable():
    """Enable robot power."""
    print("Enabling robot")
    robot.RobotEnable(state=1)


def mode():
    """Switch between automatic and manual mode."""
    print("Automatic mode")
    robot.Mode(state=0)
    time.sleep(3)
    print("Manual mode")
    robot.Mode(state=1)


# rpc()
# getsdkversion()
# getcontrollerip()
# dragteachswitch()
# isindragteach()
# robotenable()
# mode()
