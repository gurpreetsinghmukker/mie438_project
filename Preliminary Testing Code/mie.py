import matplotlib.pyplot as plt
import numpy as np

BASE_MOTOR_OFFSET = -9
SHOULDER_MOTOR_OFFSET = +6
ELBOW_MOTOR_OFFSET = 0
GRIPPER_MOTOR_OFFSET = +5


BASE_MOTOR_MIN = 0
SHOULDER_MOTOR_MIN = 0
ELBOW_MOTOR_MIN = 30
GRIPPER_MOTOR_MIN = 30

LINK0 = 9.1
LINK1 = 10.45
LINK2 = 14.64    
LINK3 = 18.5

def check_angle_limits(alpha, beta, zeta, gamma):

    print(f'alpha: {alpha*180/np.pi}, beta: {beta*180/np.pi}, zeta: {zeta*180/np.pi}, gamma: {gamma*180/np.pi}')

    if not (((alpha) >= (0+SHOULDER_MOTOR_OFFSET+SHOULDER_MOTOR_MIN)*np.pi/180) and (alpha <= (180+SHOULDER_MOTOR_OFFSET)*np.pi/180)):
        print('Shoulder motor angle out of limits')
        return False
    if not (((beta) >= (0+ELBOW_MOTOR_OFFSET+ELBOW_MOTOR_MIN)*np.pi/180) and (beta <= (180+ELBOW_MOTOR_OFFSET)*np.pi/180)):
        print('Elbow motor angle out of limits')
        return False
    if not(((zeta) >= (0+GRIPPER_MOTOR_OFFSET+GRIPPER_MOTOR_MIN)*np.pi/180) and (zeta <= (180+GRIPPER_MOTOR_OFFSET)*np.pi/180)):
        print('Gripper motor angle out of limits')
        return False
    if not(((gamma) >= (0+BASE_MOTOR_OFFSET+BASE_MOTOR_MIN)*np.pi/180) and (gamma <= (180+BASE_MOTOR_OFFSET)*np.pi/180)):
        print('Base motor angle out of limits')
        return False
    return True

def calculate_trajectory(p0, pf, v0, vf, tf, steps):
    
    # Coefficients of the cubic trajectory
    a = (2*p0 - 2*pf + tf*v0 + tf*vf) / tf**3
    b = (-3*p0 + 3*pf - 2*tf*v0 - tf*vf) / tf**2
    c = v0
    d = p0

    t = np.linspace(0, tf, steps)
    
    p = a*t**3 + b*t**2 + c*t + d
    
    v = 3*a*t**2 + 2*b*t + c
    
    a = 6*a*t + 2*b
    
    return t, p, v, a

def calc_link1_vec(alpha, gamma, link1):
    x_y = link1*np.cos(alpha - np.pi/2)
    x = x_y*np.cos(gamma)
    y = x_y*np.sin(gamma)
    z = link1*np.sin(alpha - np.pi/2)
    return np.array([x,y,z])

def calc_link2_vec(alpha, beta, gamma, link2):
    x_y = link2*np.cos(((3/2)*np.pi- alpha - beta))
    z = - link2*np.sin(((3/2)*np.pi- alpha - beta))
    x = x_y*np.cos(gamma)
    y = x_y*np.sin(gamma)
    return np.array([x,y,z])

def theta(vec):
    rad = np.arctan2(vec[2],np.linalg.norm(vec[:2], 2))
    return rad

def calculate_angles(vec,link0, link1, link2, link3):

    link0_vec = np.array([0,0,link0])
    link3_vec = np.array([0,0,-link3])
   # Calculate the angle alpha between link1 and r 
    xy = np.linalg.norm(vec[:2], 2)
    r_dd_vec = vec - link0_vec - link3_vec
    # r_d_vec = r_dd_vec + link0_vec
    z_vec = vec - link0_vec
    z = np.linalg.norm(z_vec, 2)
    # if z > link1 + link2 + link3:
    #     print('Error: Vector length is greater than the sum of the links')
    #     return (0,0,0,0)
    
    if np.linalg.norm(r_dd_vec,2) > link1 + link2:
        alpha_dd = 0
        alpha_d = np.arccos(((link1 + link2)**2 + z**2 - link3**2) / (2*(link1 + link2)*z)) + alpha_dd
        r = np.linalg.norm(vec, 2)
        alpha = (np.arccos(((z)**2 + link0**2 - r**2) / (2*z*link0)) + alpha_d)

        beta = np.pi
        zeta = np.arccos(((link1+link2)**2 + link3**2 - z**2) / (2*(link1+link2)*link3))
        if xy == 0:
            gamma = np.pi/2
        else:
            gamma = np.arctan2(vec[1],vec[0])

    else:
        r_dd = np.linalg.norm(r_dd_vec, 2)
        alpha_dd = np.arccos((link1**2 + r_dd**2 - link2**2) / (2*link1*r_dd))
        alpha_d = np.arccos((r_dd**2 + z**2 - link3**2) / (2*z*r_dd)) +alpha_dd
        r = np.linalg.norm(vec, 2)
        alpha = (np.arccos(((z)**2 + link0**2 - r**2) / (2*z*link0)) + alpha_d)
        beta = np.arccos((link1**2 + link2**2 - r_dd**2) / (2*link1*link2))
        zeta = 3*np.pi - alpha - beta - np.pi

        if xy == 0:
            gamma = np.pi/2
        else:
            gamma = np.arctan2(vec[1],vec[0])

    return (alpha - np.pi/2, beta, zeta, gamma)

def plot_vecs(v,link0, link1, link2, link3):

    # ax.quiver(0, 0, 0, v[0], v[1], v[2], color='k', ls = 'dashed', arrow_length_ratio=0.1)
    
    alpha, beta, zeta, gamma = calculate_angles(v,link0, link1,link2,link3)

    link0_vec = np.array([0,0,link0])
    link1_vec = calc_link1_vec(alpha+np.pi/2,gamma,link1)

    if beta == np.pi:
        link2_vec = link1_vec/np.linalg.norm(link1_vec, 2)*link2
        link3_vec = calc_link2_vec(alpha+np.pi/2,zeta,gamma,link3)
    else:
        link2_vec = calc_link2_vec(alpha+np.pi/2,beta,gamma,link2)
        link3_vec = np.array([0,0,-link3])
    
    # print(f'alpha: {alpha*180/np.pi}, beta: {beta*180/np.pi}, zeta: {zeta*180/np.pi}, gamma: {gamma*180/np.pi}, zeta: {zeta*180/np.pi}')
    # #print angles in radians
    # print(f'alpha: {alpha}, beta: {beta}, zeta: {zeta}, gamma: {gamma}')
    # print(f'link0_vec: {link0_vec}')
    # print(f'link1_vec: {link1_vec}')
    # print(f'link2_vec: {link2_vec}')
    # print(f'link3_vec: {link3_vec}')
    # print(f'link0_vec_norm: {np.linalg.norm(link0_vec)}')
    # print(f'link1_vec_norm: {np.linalg.norm(link1_vec)}')
    # print(f'link2_vec_norm: {np.linalg.norm(link2_vec)}')
    # print(f'link_vec_norm: {np.linalg.norm(link3_vec)}')
    # print(f'link_sum_norm: {np.linalg.norm(link0_vec + link1_vec + link2_vec + link3_vec)}')
    # print(f'link_sum_norm: {np.linalg.norm(link1_vec + link2_vec)}')

    ax.quiver(0, 0, 0, link0_vec[0], link0_vec[1], link0_vec[2], color='y', ls = 'dashed', arrow_length_ratio=0.0001)
    # print(link0_vec[0], link0_vec[1], link0_vec[2], link0_vec[0]+link1_vec[0], link0_vec[1]+link1_vec[1], link0_vec[2]+link1_vec[2])
    ax.quiver(link0_vec[0],link0_vec[1] ,link0_vec[2],link1_vec[0], link1_vec[1], link1_vec[2], color='r', ls = 'dashed', arrow_length_ratio=0.0001)
    # print(link1_vec[0], link1_vec[1], link1_vec[2], link1_vec[0]+link2_vec[0], link1_vec[1]+link2_vec[1], link1_vec[2]+link2_vec[2])
    ax.quiver(link1_vec[0]+link0_vec[0], link1_vec[1]+link0_vec[1], link1_vec[2]+link0_vec[2], link2_vec[0], link2_vec[1], link2_vec[2], color='g', ls = 'dashed', arrow_length_ratio=0.0001)
    # print(link2_vec[0], link2_vec[1], link2_vec[2], link2_vec[0]+link3_vec[0], link2_vec[1]+link3_vec[1], link2_vec[2]+link3_vec[2])
    ax.quiver(link0_vec[0]+link1_vec[0]+link2_vec[0], link0_vec[1]+link1_vec[1]+link2_vec[1],link0_vec[2]+ link1_vec[2]+link2_vec[2], link3_vec[0], link3_vec[1], link3_vec[2], color='b', ls = 'dashed', arrow_length_ratio=0.0001)
    # print(link3_vec[0], link3_vec[1], link3_vec[2])
    
    # ax.set_xlim([min(2*min(link1_vec[0], link2_vec[0], v[0]), -1), max(2*max(link1_vec[0], link2_vec[0], v[0]), +1)])
    # ax.set_ylim([min(2*min(link1_vec[1], link2_vec[1], v[1]), -1), max(2*max(link1_vec[1], link2_vec[1], v[1]), +1)])
    # ax.set_zlim([0, max(4*max(link1_vec[2], link2_vec[2], v[2]), +1)])

    # Set the labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # plt.show()
    return alpha, beta, zeta, gamma

    
if __name__=='__main__':

    # LINK0 = 9.1
    # LINK1 = 10.45
    # LINK2 = 14.64    
    # LINK3 = 15
#define LINK_0_LEN 1
#define LINK_1_LEN 1
#define LINK_2_LEN 1
#define LINK_3_LEN 0.25

    simple_plot = plt.figure()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([0, 20])

    pos_i = np.array([7, 5 , 1])
    ax.scatter(pos_i[0],pos_i[1],pos_i[2], color='orange')

    plot_vecs(pos_i, LINK0, LINK1, LINK2, LINK3)


    pos_f = np.array([2, 8 ,8])
    plot_vecs(pos_f, LINK0, LINK1, LINK2, LINK3)
    ax.scatter(pos_f[0],pos_f[1],pos_f[2], color='brown')

    if  check_angle_limits(*calculate_angles(pos_f, LINK0, LINK1, LINK2, LINK3)):
        plot_vecs(pos_f, LINK0, LINK1, LINK2, LINK3)

        vec_i_f = pos_f - pos_i
        steps = 10
        # intermediate_vecs = pos_i + np.arange(1/steps,1+1/steps,1/steps).reshape(-1,1)*vec_i_f
        # print(intermediate_vecs)
        p = np.zeros((steps,3))
        v = np.zeros((steps,3))
        a = np.zeros((steps,3))
        t = np.zeros(steps)
        for i in range(3):
            t, p[:,i], v[:,i], a[:,i] = calculate_trajectory(pos_i[i], pos_f[i], 0, 0, 1, steps)

        # print(p)
        # print(v)
        # simple_plot.plot(t, p[:,0], label='X')
        # simple_plot.plot(t, p[:,1], label='Y')
        # simple_plot.plot(t, p[:,2], label='Z')

        for i in range(steps):
            print(plot_vecs(p[i], LINK0, LINK1, LINK2, LINK3))
            print()
        # ax.quiver(pos_i[0], pos_i[1], pos_i[2], vec_i_f[0], vec_i_f[1], vec_i_f[2], color='m', ls = 'dashed', arrow_length_ratio=0.1)
        ax.set_aspect('equal', adjustable='box')
    
    else:
        print('Final position out of limits')
    # simple_plot.show()
    plt.show()

    
    









    