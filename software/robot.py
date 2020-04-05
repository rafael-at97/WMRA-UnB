import sys

import_path = "/home/rafael_at/Documents/Estudos/TG/Codes/robopy/robopy"
sys.path.insert(0, import_path)
print("Appending import path: " + import_path)

from robot.link import Revolute, Link, SerialLink
from math import pi

###            Denavit-Hartenberg parameters           ###
# 
# i -> Joint number, starting from 0
# alpha(i) -> Angle from axis Z(i) to Z(i+1), along X(i)
# a(i) -> Distance from axis Z(i) to Z(i+1), along X(i)
# d(i) -> Distance from X(i-1) to X(i), along Z(i)
# theta(i) -> Angle from X(i-1) to X(i), along Z(i)
#
#  ------------------------------------------------------
# |   i   |  alpha(i-1) |   a(i-1)   |  d(i)  | theta(i) |
#  -------------------------------------------------------  
#  -------------------------------------------------------
# |   1   |      0      |      0     |    0   | theta(1) |
#  -------------------------------------------------------
# |   2   |     +90     |   0.014    |    0   | theta(2) |
#  ------------------------------------------------------- 
# |   3   |      0      |    0.35    |  0.069 | theta(3) |
#  ------------------------------------------------------  
# |   4   |      0      |    0.35    | -0.069 | theta(4) |
#  ------------------------------------------------------
# |   5   |     +90     |   0.165    |    0   | theta(5) |
#  ------------------------------------------------------
# |   6   |     -90     |      0     |  0.198 | theta(6) |
#  ------------------------------------------------------
#

if __name__ == "__main__":
    links = []
    links.append(Revolute(alpha=0,     A=0,     D=0,                    mdh="Modified", m=0,     r=[-0.03, -0.057, 0.008], I=[[0.002, 0.001,   0.0],
                                                                                                                              [0.001, 0.002,   0.0],
                                                                                                                              [  0.0,   0.0, 0.003]]))

    links.append(Revolute(alpha=pi/2,  A=0.014, D=0,                    mdh="Modified", m=1.236, r=[0.26, 0.008, -0.013],  I=[[0.002,   0.0,   0.0],
                                                                                                                              [  0.0, 0.015,   0.0],
                                                                                                                              [  0.0,   0.0, 0.015]]))

    links.append(Revolute(alpha=0,     A=0.35,  D=0.069,                mdh="Modified", m=1.236, r=[0.26, -0.008, 0.013],  I=[[0.002,   0.0,   0.0],
                                                                                                                              [  0.0, 0.015,   0.0],
                                                                                                                              [  0.0,   0.0, 0.015]]))
                                                                                                                             
    links.append(Revolute(alpha=0,     A=0.35,  D=-0.069,               mdh="Modified", m=0.63,  r=[0.116, 0.032, 0],      I=[[0.002, 0.001,   0.0],
                                                                                                                              [0.001, 0.002,   0.0],
                                                                                                                              [  0.0,   0.0, 0.004]]))

    links.append(Revolute(alpha=pi/2,  A=0.165, D=0,      offset=-pi/2, mdh="Modified", m=0.49,  r=[0, 0.1, 0],            I=[[0.002, 0.0,   0.0],
                                                                                                                              [  0.0, 0.0,   0.0],
                                                                                                                              [  0.0, 0.0, 0.002]]))

    links.append(Revolute(alpha=-pi/2, A=0,     D=0.198,                mdh="Modified", m=0,     r=[0, 0, 0],              I=[[0, 0, 0],
                                                                                                                              [0, 0, 0],
                                                                                                                              [0, 0, 0]]))

    robot = SerialLink(links)

    #robot.display()

    print(robot.rne([0, 0, 0, 0, 0, 0]))