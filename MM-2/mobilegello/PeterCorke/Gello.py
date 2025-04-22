#!/usr/bin/env python

from math import pi
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, RevoluteMDH, PrismaticMDH


class GELLO(DHRobot):
    """
    Create model of GELLO manipulator        
        Class that models Active Arm - resembling UR3e

    :notes:
       .
    :references: 
    """

    def __init__(self):

        deg = pi/180

        a=[0,-127,-105,0,0,0]
        alpha=[pi/2,0,0,pi/2,-pi/2,0]
        d=[57,0,0,56,47,45]

        qlim_array = [[0*deg, 360*deg],
                      [-270*deg, 90*deg],
                      [-270*deg, 90*deg],
                      [-270*deg, 90*deg],
                      [-90*deg, 270*deg],
                      [-180*deg, 180*deg]]
        
        links = []

        # mass=[0.35,0.35,0.35,0.35,0.35,0.35]  # Masses of links [Not Mandatory]
    
        # center_of_mass = [  # Center of Mass [Not Mandatory]
        #     [0, -0.02, 0],
        #     [0.13, 0, 0.1157],
        #     [0.05, 0, 0.0238],
        #     [0, 0, 0.01],
        #     [0, 0, 0.01],
        #     [0, 0, -0.02],
        # ]    
        # _______________________________ SAMPLE FROM THE SITE 
        # L0 = RevoluteDH(
        #     d=0,          # link length (Dennavit-Hartenberg notation)
        #     a=0,          # link offset (Dennavit-Hartenberg notation)
        #     alpha=pi/2,   # link twist (Dennavit-Hartenberg notation)
        #     I=[0, 0.35, 0, 0, 0, 0],  # inertia tensor of link with respect to
        #                               # center of mass I = [L_xx, L_yy, L_zz,
        #                               # L_xy, L_yz, L_xz]
        #     r=[0, 0, 0],  # distance of ith origin to center of mass [x,y,z]
        #                   # in link reference frame
        #     m=0,          # mass of link
        #     Jm=200e-6,    # actuator inertia
        #     G=-62.6111,   # gear ratio
        #     B=1.48e-3,    # actuator viscous friction coefficient (measured
        #                   # at the motor)
        #     Tc=[0.395, -0.435],  # actuator Coulomb friction coefficient for
        #                          # direction [-,+] (measured at the motor)
        #     qlim=[-160*deg, 160*deg])    # minimum and maximum joint angle


        # Create the links
        for j in range(6):
            link = RevoluteDH(
                # d=d[j], a=a[j], alpha=alpha[j], m=mass[j], r=center_of_mass[j], G=1
                d=d[j], a=a[j], alpha=alpha[j], G=1, qpos=[-10,10]
            )
            links.append(link)
        
        # Create the robot
        super().__init__(
            links,
            name="GELLO",
            manufacturer="RAASLAB",
            )
        
        # self._MYCONFIG = np.array([3.14,-1.57,-1.57,0,1.57,3.14])
        self._MYCONFIG = np.array([3.14,-1.57, -0.785 ,-0.785,1.57,3.14])

        @property
        def MYCONFIG(self):
            return self._MYCONFIG
        


        

if __name__ == '__main__':

    robot = GELLO()
    print(robot)