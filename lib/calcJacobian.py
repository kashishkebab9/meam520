import numpy as np
from lib.calculateFK import FK
import math

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    l_0 = .141
    l_1 = .192
    l_2 = .195
    l_3 = .0825
    l_3_z = .121
    l_4_x = .0825
    l_4_y = .125
    l_5_z = .259
    l_5_x_y = .015
    l_6_z = .015
    l_6_x = .088
    l_6_y = .051
    l_7 = .159

    T0e = np.identity(4)
    T_W_0 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, l_0], [0, 0, 0, 1]])

    T_0_1 = np.matrix([[math.cos(q[0]), 0, -math.sin(q[0]),  0], 
                       [math.sin(q[0]), 0, math.cos(q[0]),   0], 
                       [      0,       -1,        0,       l_1], 
                       [      0,        0,        0,         1]])
    T_W_1 = T_W_0 @ T_0_1
    

    T_1_2 = np.matrix([[math.cos(q[1]), 0, math.sin(q[1]),   l_2*math.sin(q[1])], 
                       [math.sin(q[1]), 0, -math.cos(q[1]), -l_2*math.cos(q[1])], 
                       [      0,        1,        0,                          0], 
                       [      0,        0,        0,                         1]])
    T_W_2 = T_W_0 @ T_0_1 @ T_1_2

    T_2_3 = np.matrix([[math.cos(q[2]), 0, math.sin(q[2]),  l_3*math.cos(q[2])], 
                       [math.sin(q[2]), 0, -math.cos(q[2]), l_3*math.sin(q[2])], 
                       [      0,        1,        0,                     l_3_z], 
                       [      0,        0,        0,                        1]])

    T_W_3 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3

    T_3_4 = np.matrix([[math.cos(q[3]), 0, -math.sin(q[3]), -l_4_x*math.cos(q[3]) - l_4_y*math.sin(q[3])], 
                       [math.sin(q[3]), 0, math.cos(q[3]), -l_4_x*math.sin(q[3]) + l_4_y*math.cos(q[3])], 
                       [      0,       -1,        0,                         0], 
                       [      0,        0,        0,                        1]])

    T_W_4 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4

    T_4_5 = np.matrix([[math.cos(q[4]), 0, math.sin(q[4]), -l_5_x_y*math.sin(q[4])], 
                       [math.sin(q[4]), 0, -math.cos(q[4]), l_5_x_y*math.cos(q[4])], 
                       [      0,        1,        0,                         l_5_z], 
                       [      0,        0,        0,                            1]])

    T_W_5 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5

    T_5_6 = np.matrix([[math.cos(q[5]), 0, math.sin(q[5]), l_6_x*math.cos(q[5]) + l_6_y*math.sin(q[5])], 
                       [math.sin(q[5]), 0, -math.cos(q[5]),l_6_x*math.sin(q[5]) - l_6_y*math.cos(q[5])], 
                       [      0,        1,        0,                         l_6_z], 
                       [      0,        0,        0,                            1]])

    T_W_6 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6

    T_6_7 = np.matrix([[math.cos(q[6] - math.pi/4), -math.sin(q[6] - math.pi/4), 0,    0], 
                       [math.sin(q[6] - math.pi/4),  math.cos(q[6] - math.pi/4), 0,    0], 
                       [              0,                           0,            1,  l_7], 
                       [              0,                           0,            0,    1]])
    T_W_7 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ T_6_7
    o_e = T_W_7[:3, 3];

    o_0 = T_W_0[:3, 3]
    right_0 = o_e - o_0
    right_0 = np.squeeze(np.asarray(right_0))
    z_0 = T_W_0[:3, 2]
    z_0 = np.squeeze(np.asarray(z_0))
    jk_v_0 = np.cross(z_0, right_0)
    jk_w_0 = T_W_0[:3, 2]
    jk_w_0 = np.reshape(np.asarray(jk_w_0), (3,))

    o_1 = T_W_1[:3, 3]
    right_1 = o_e - o_1
    right_1 = np.squeeze(np.asarray(right_1))
    z_1 = T_W_1[:3, 2]
    z_1 = np.squeeze(np.asarray(z_1))
    jk_v_1 = np.cross(z_1, right_1)
    jk_w_1 = T_W_1[:3, 2]
    jk_w_1 = np.reshape(np.asarray(jk_w_1), (3,))

    o_2 = T_W_2[:3, 3]
    right_2 = o_e - o_2
    right_2 = np.squeeze(np.asarray(right_2))
    z_2 = T_W_2[:3, 2]
    z_2 = np.squeeze(np.asarray(z_2))
    jk_v_2 = np.cross(z_2, right_2)
    jk_w_2 = T_W_2[:3, 2]
    jk_w_2 = np.reshape(np.asarray(jk_w_2), (3,))

    o_3 = T_W_3[:3, 3]
    right_3 = o_e - o_3
    right_3 = np.squeeze(np.asarray(right_3))
    z_3 = T_W_3[:3, 2]
    z_3 = np.squeeze(np.asarray(z_3))
    jk_v_3 = np.cross(z_3, right_3)
    jk_w_3 = T_W_3[:3, 2]
    jk_w_3 = np.reshape(np.asarray(jk_w_3), (3,))

    o_4 = T_W_4[:3, 3]
    right_4 = o_e - o_4
    right_4 = np.squeeze(np.asarray(right_4))
    z_4 = T_W_4[:3, 2]
    z_4 = np.squeeze(np.asarray(z_4))
    jk_v_4 = np.cross(z_4, right_4)
    jk_w_4 = T_W_4[:3, 2]
    jk_w_4 = np.reshape(np.asarray(jk_w_4), (3,))

    o_5 = T_W_5[:3, 3]
    right_5 = o_e - o_5
    right_5 = np.squeeze(np.asarray(right_5))
    z_5 = T_W_5[:3, 2]
    z_5 = np.squeeze(np.asarray(z_5))
    jk_v_5 = np.cross(z_5, right_5)
    jk_w_5 = T_W_5[:3, 2]
    jk_w_5 = np.reshape(np.asarray(jk_w_5), (3,))

    o_6 = T_W_6[:3, 3]
    right_6 = o_e - o_6
    right_6 = np.squeeze(np.asarray(right_6))
    z_6 = T_W_6[:3, 2]
    z_6 = np.squeeze(np.asarray(z_6))
    jk_v_6 = np.cross(z_6, right_6)
    jk_w_6 = T_W_6[:3, 2]
    jk_w_6 = np.reshape(np.asarray(jk_w_6), (3,))

    J[:3, 0] = jk_v_0
    J[3:6, 0] = jk_w_0
    J[:3, 1] = jk_v_1
    J[3:6, 1] = jk_w_1
    J[:3, 2] = jk_v_2
    J[3:6, 2] = jk_w_2
    J[:3, 3] = jk_v_3
    J[3:6, 3] = jk_w_3
    J[:3, 4] = jk_v_4
    J[3:6, 4] = jk_w_4
    J[:3, 5] = jk_v_5
    J[3:6, 5] = jk_w_5
    J[:3, 6] = jk_v_6
    J[3:6, 6] = jk_w_6
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q= np.array([0, 0, 0, 0, 0, 0, 0])
    print(np.round(calcJacobian(q),3))
