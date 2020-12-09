from robopy.robot.link import Revolute, Link, SerialLink
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
# |   1   |      0      |      0     |  0.187 | theta(1) |
#  -------------------------------------------------------
# |   2   |     +90     |   0.014    |    0   | theta(2) |
#  ------------------------------------------------------- 
# |   3   |      0      |    0.35    |    0   | theta(3) |
#  ------------------------------------------------------  
# |   4   |      0      |    0.35    |    0   | theta(4) |
#  ------------------------------------------------------
# |   5   |     +90     |   0.165    |    0   | theta(5) |
#  ------------------------------------------------------
# |   6   |     -90     |      0     |  0.198 | theta(6) |
#  ------------------------------------------------------
#

if __name__ == "__main__":
    links = []
    links.append(Revolute(alpha=0,     A=0,     D=0.187,                mdh="Modified", m=0.674, r=[-0.035, -0.062, 0.009], I=[[0.002, 0.001,   0.0],
                                                                                                                               [0.001, 0.002,   0.0],
                                                                                                                               [  0.0,   0.0, 0.002]]))

    links.append(Revolute(alpha=pi/2,  A=0.014, D=0,                    mdh="Modified", m=1.257, r=[0.231, 0.007, -0.015],  I=[[0.002,   0.0, -0.001],
                                                                                                                               [ 0.0,  0.021,  0.0  ],
                                                                                                                               [-0.001,  0.0,  0.020]]))

    links.append(Revolute(alpha=0,     A=0.350, D=0,                    mdh="Modified", m=1.235, r=[0.235, -0.008, 0.090],  I=[[0.002,   0.0, 0.002],
                                                                                                                               [  0.0, 0.019,   0.0],
                                                                                                                               [0.002,   0.0, 0.019]]))
                                                                                                                             
    links.append(Revolute(alpha=0,     A=0.350, D=0,                    mdh="Modified", m=0.672, r=[0.097, 0.030, 0.005],   I=[[0.002, 0.001,   0.0],
                                                                                                                               [0.001, 0.003,   0.0],
                                                                                                                               [  0.0,   0.0, 0.005]]))

    links.append(Revolute(alpha=pi/2,  A=0.165, D=0,      offset=-pi/2, mdh="Modified", m=0.537, r=[0, 0.091, 0],           I=[[0.002, 0.0,   0.0],
                                                                                                                               [  0.0, 0.0,   0.0],
                                                                                                                               [  0.0, 0.0, 0.002]]))

    links.append(Revolute(alpha=-pi/2, A=0,     D=0.198,                mdh="Modified", m=1.200, r=[0, 0, 0.1],             I=[[0, 0, 0],
                                                                                                                               [0, 0, 0],
                                                                                                                               [0, 0, 0]]))

    base = [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1.0]]

    robot = SerialLink(links, base=base)

    robot.display()

    print(robot.rne([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0.07, 0.07, 0.07, 0.07, 0.07, 0.07]))

    input("end: ")