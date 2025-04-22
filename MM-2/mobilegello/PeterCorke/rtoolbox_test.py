import roboticstoolbox as rtb
from spatialmath import SE3
from Gello import GELLO
import numpy as np

# create the robot
robot = GELLO()
print(robot)

# check the joint configuration
print(robot._MYCONFIG)    # check that the joint configuration works
testcase_1 = np.array([3.14,-1.57, -0.785 ,-0.785,1.57,3.14])
# testcase_1 = np.array([3.17, -1.61, -1.64, 0.09, -0.14, 3.02])
# find forward kinematics
# Te = robot.fkine(robot._MYCONFIG)  # forward kinematics
Te = robot.fkine(testcase_1)  # forward kinematics
print(Te)

# find inverse kinematics
# Tep = SE3.Trans(-105,101,231) * SE3.OA([1,0,0],[0,0,-1])
Tep = SE3.Trans(Te.t[0],Te.t[1],Te.t[2]) * SE3.OA(Te.R[:, 1],Te.R[:, 2])

# sol = robot.ik_LM(Tep, q0=robot._MYCONFIG)         # solve IK
sol = robot.ik_NR(Tep, q0=testcase_1)         # solve IK

print(sol[0])                 # display the solution
# print(robot.qlim)

q_pickup = sol[0]
print(robot.fkine(q_pickup))

# qt = rtb.jtraj(robot._MYCONFIG, sol[0], 10)

# print(qt.q)
# robot.plot(qt.q, backend='pyplot', movie='gello1.gif')