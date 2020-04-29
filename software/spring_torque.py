import numpy as np
from math import pi, cos, sin, sqrt

if __name__ == "__main__":

    dist = 50*sqrt(2)
    radius = 10
    L0 = dist - radius

    theta = 3*pi/4
    max_length = sqrt(pow(L0 + radius, 2) + pow(radius, 2) - 2*radius*(L0 + radius)*cos(theta))

    theta = -pi/4
    t1 = (1 - L0/sqrt(pow(L0 + radius, 2) + pow(radius, 2) - 2*radius*(L0 + radius)*cos(theta)))*(dist)*radius*sin(theta)/1000

    theta = pi/2
    t2 = (1 - L0/sqrt(pow(L0 + radius, 2) + pow(radius, 2) - 2*radius*(L0 + radius)*cos(theta)))*(dist)*radius*sin(theta)/1000

    A = np.array([t1, t1, t2, t2])
    B = np.array([0.97, -17, 20.23, 2.71])

    K = pow(np.matmul(A, A), -1)*np.matmul(A, B)

    print("Max deformation: " + str(max_length*100/L0 - 100))
    print("K: " + str(K))