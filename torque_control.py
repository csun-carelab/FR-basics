import Robot

print("Robot module path:", Robot.__file__)
print("Has ServoJTStart? ", hasattr(Robot.RPC, "ServoJTStart"))
print("Has ServoJT?      ", hasattr(Robot.RPC, "ServoJT"))
print("Has ServoJTEnd?   ", hasattr(Robot.RPC, "ServoJTEnd"))

# connect to robot
robot = Robot.RPC('192.168.58.2')

# switch to compliant mode
robot.DragTeachSwitch(1)

# start joint torque control mode
robot.ServoJTStart()

# apply a small amount of toque on the first joint
torques = [0.1, 0, 0, 0, 0, 0]
interval = 0.005

input("Keep one hand on the E-STOP. Press Enter to start.")

for _ in range(10):
    err = robot.ServoJT(torques, interval)
    if err != 0:
        print("ServoJT error:", err)
        break

robot.ServoJTEnd()
robot.DragTeachSwitch(0)
robot.CloseRPC()
