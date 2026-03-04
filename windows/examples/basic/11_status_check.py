import Robot
import time

# Establish a connection to the robot controller.
robot = Robot.RPC("192.168.58.2")

JP1 = [117.408, -86.777, 81.499, -87.788, -92.964, 92.959]
DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]

JP2 = [72.515, -86.774, 81.525, -87.724, -91.964, 92.958]
DP2 = [-65.169, -529.17, 518.018, -177.189, 3.119, 69.556]


def getrobotinstallangle():
    """Get robot installation angle."""
    robot.SetRobotInstallAngle(yangle=90, zangle=90)
    error, installangle = robot.GetRobotInstallAngle()
    print("GetRobotInstallAngle return ", error)
    print("Current installation angle:", installangle)


def getinversekin():
    """Inverse kinematics test."""
    error, jp = robot.GetInverseKin(type=0, desc_pos=DP1)
    print("GetInverseKin return ", error)
    print("Inverse kinematics result:", jp)
    print("Reference joints:", JP1)


def getinversekinhassolution():
    """Check whether inverse kinematics has a valid solution."""
    error, has_solution = robot.GetInverseKinHasSolution(type=0, desc_pos=DP1, joint_pos_ref=JP1)
    print("GetInverseKinHasSolution return ", error)
    print("Has solution:", has_solution)


def getforwardkin():
    """Forward kinematics test."""
    error, dp = robot.GetForwardKin(joint_pos=JP2)
    print("GetForwardKin return ", error)
    print("Forward kinematics result:", dp)
    print("Reference Cartesian pose:", DP2)


def getversion():
    """Get firmware and hardware version information."""
    error, *hard_versions = robot.GetSlaveHardVersion()
    print("GetSlaveHardVersion return ", error)
    print("Hardware info:", *hard_versions)
    error, *firm_versions = robot.GetSlaveFirmVersion()
    print("GetSlaveFirmVersion return ", error)
    print("Firmware info:", *firm_versions)


def getsshkeygen():
    """Get SSH public key."""
    error, sshkeygen = robot.GetSSHKeygen()
    print("GetSSHKeygen return ", error)
    print("SSH public key:", sshkeygen)


def gettargetpayload():
    """Get current target payload and center of gravity."""
    error, targetpayload = robot.GetTargetPayload()
    print("GetTargetPayload return ", error)
    print("Current payload:", targetpayload)
    error, targetpos = robot.GetTargetPayloadCog()
    print("Current payload center of gravity:", targetpos)


def getrobotcurjointsconfig():
    """Get current joint configuration."""
    error, robotcurjointsconfig = robot.GetRobotCurJointsConfig()
    print("GetRobotCurJointsConfig return ", error)
    print("Current joint configuration:", robotcurjointsconfig)


def getsystemclock():
    """Get system clock."""
    error, systemclock = robot.GetSystemClock()
    print("GetSystemClock return ", error)
    print("System time:", systemclock)


def getdefaulttransvel():
    """Get default translation velocity."""
    error, defaulttransvel = robot.GetDefaultTransVel()
    print("GetDefaultTransVel return ", error)
    print("Default translation velocity:", defaulttransvel)


def gettcpoffset():
    """Get current TCP offset."""
    error, tcpoffset = robot.GetTCPOffset()
    print("GetTCPOffset return ", error)
    print("Current TCP offset:", tcpoffset)


def getwobjoffset():
    """Get current work-object offset."""
    error, wobjoffset = robot.GetWObjOffset()
    print("GetWObjOffset return ", error)
    print("Current work-object offset:", wobjoffset)


def getjointsoftlimitdeg():
    """Get joint soft limits in degrees."""
    error, jointsoftlimitdeg = robot.GetJointSoftLimitDeg()
    print("GetJointSoftLimitDeg return ", error)
    print("Joint soft limits (deg):", jointsoftlimitdeg)


def getrobotmotiondone():
    """Query whether robot motion is complete."""
    error, robotmotiondone = robot.GetRobotMotionDone()
    print("GetRobotMotionDone return ", error)
    print("Motion complete:", robotmotiondone)


def getrobotteachingpoint():
    """Get teaching point data by name."""
    error, robotteachingpoint = robot.GetRobotTeachingPoint(name="P1")
    print("GetRobotTeachingPoint return ", error)
    print("Teaching point data:", robotteachingpoint)


# getrobotinstallangle()
# getinversekin()
# getinversekinhassolution()
# getforwardkin()
# getversion()
# getsshkeygen()
# gettargetpayload()  # Disabled by default
# getrobotcurjointsconfig()
# getsystemclock()
# getdefaulttransvel()
# gettcpoffset()
# getwobjoffset()
# getjointsoftlimitdeg()
# getrobotmotiondone()
# getrobotteachingpoint()
