import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_trajectory_3(x0, xf, v0, t_f, steps):
    d = x0
    c = v0
    b = (3*(xf-x0) - 2*v0*t_f) / (t_f**2)
    a = (-v0-2*b*t_f)/(3*(t_f**2))

    t = np.linspace(0, t_f, steps)
    x = a*t**3 + b*t**2 + c*t + d
    v = 3*a*t**2 + 2*b*t + c

    return t, x, v

def calculate_trajectory_linear(p0, pf, v, steps):
    t = np.linspace(0, (pf-p0)/v, steps)
    p = p0+v*t
    v = np.ones(steps)*v
    return t, p, v


def calculate_trajectory(x0, xf, v0, t_maxv, steps):
    # Coefficients of the cubic trajectory
    
    d = x0
    c = v0
    a = (xf - x0 - t_maxv*v0) / (-2*(t_maxv**3))
    b = -3*a*t_maxv

    # Time vector
    t = np.linspace(0, t_maxv, steps)

    # Position vector
    x = a*t**3 + b*t**2 + c*t + d

    # Velocity vector
    v = 3*a*t**2 + 2*b*t + c

    return t, x, v  

if __name__=="__main__":

    t1, x1, v1 = calculate_trajectory(0, 10, 0, 2, 10)
    print(v1[-1])
    t2, x2, v2 = calculate_trajectory_linear(10, 20, v1[-1], 10)
    t3, x3, v3 = calculate_trajectory_3(20, 30, v2[-1], 2, 10)

    t = np.concatenate((t1, t2+2, t3+2+t2[-1]))
    x = np.concatenate((x1, x2, x3))
    v = np.concatenate((v1, v2, v3))

    plt.plot(t, x, label='Position')
    plt.plot(t, v, label='Velocity')
    plt.legend()
    plt.show()




