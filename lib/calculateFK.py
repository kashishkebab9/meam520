import numpy as np
import math 
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))

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
        jointPositions[0,0] = T_W_0[0,3]
        jointPositions[0,1] = T_W_0[1,3]
        jointPositions[0,2] = T_W_0[2,3]

        T_0_1 = np.matrix([[math.cos(q[0]), 0, -math.sin(q[0]),  0], 
                           [math.sin(q[0]), 0, math.cos(q[0]),   0], 
                           [      0,       -1,        0,       l_1], 
                           [      0,        0,        0,         1]])

        T_W_1 = T_W_0 @ T_0_1
        jointPositions[1,0] = T_W_1[0,3]
        jointPositions[1,1] = T_W_1[1,3]
        jointPositions[1,2] = T_W_1[2,3]

        T_1_2 = np.matrix([[math.cos(q[1]), 0, math.sin(q[1]),   l_2*math.sin(q[1])], 
                           [math.sin(q[1]), 0, -math.cos(q[1]), -l_2*math.cos(q[1])], 
                           [      0,        1,        0,                          0], 
                           [      0,        0,        0,                         1]])

        T_W_2 = T_W_0 @ T_0_1 @ T_1_2
        jointPositions[2,0] = T_W_2[0,3]
        jointPositions[2,1] = T_W_2[1,3]
        jointPositions[2,2] = T_W_2[2,3]

        T_2_3 = np.matrix([[math.cos(q[2]), 0, math.sin(q[2]),  l_3*math.cos(q[2])], 
                           [math.sin(q[2]), 0, -math.cos(q[2]), l_3*math.sin(q[2])], 
                           [      0,        1,        0,                     l_3_z], 
                           [      0,        0,        0,                        1]])

        T_W_3 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3
        jointPositions[3,0] = T_W_3[0,3]
        jointPositions[3,1] = T_W_3[1,3]
        jointPositions[3,2] = T_W_3[2,3]

        T_3_4 = np.matrix([[math.cos(q[3]), 0, -math.sin(q[3]), -l_4_x*math.cos(q[3]) - l_4_y*math.sin(q[3])], 
                           [math.sin(q[3]), 0, math.cos(q[3]), -l_4_x*math.sin(q[3]) + l_4_y*math.cos(q[3])], 
                           [      0,       -1,        0,                         0], 
                           [      0,        0,        0,                        1]])

        T_W_4 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4
        jointPositions[4,0] = T_W_4[0,3]
        jointPositions[4,1] = T_W_4[1,3]
        jointPositions[4,2] = T_W_4[2,3]

        T_4_5 = np.matrix([[math.cos(q[4]), 0, math.sin(q[4]), -l_5_x_y*math.sin(q[4])], 
                           [math.sin(q[4]), 0, -math.cos(q[4]), l_5_x_y*math.cos(q[4])], 
                           [      0,        1,        0,                         l_5_z], 
                           [      0,        0,        0,                            1]])

        T_W_5 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5
        jointPositions[5,0] = T_W_5[0,3]
        jointPositions[5,1] = T_W_5[1,3]
        jointPositions[5,2] = T_W_5[2,3]

        T_5_6 = np.matrix([[math.cos(q[5]), 0, math.sin(q[5]), l_6_x*math.cos(q[5]) + l_6_y*math.sin(q[5])], 
                           [math.sin(q[5]), 0, -math.cos(q[5]),l_6_x*math.sin(q[5]) - l_6_y*math.cos(q[5])], 
                           [      0,        1,        0,                         l_6_z], 
                           [      0,        0,        0,                            1]])

        T_W_6 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6
        jointPositions[6,0] = T_W_6[0,3]
        jointPositions[6,1] = T_W_6[1,3]
        jointPositions[6,2] = T_W_6[2,3]

        T_6_7 = np.matrix([[math.cos(q[6] - math.pi/4), -math.sin(q[6] - math.pi/4), 0,    0], 
                           [math.sin(q[6] - math.pi/4),  math.cos(q[6] - math.pi/4), 0,    0], 
                           [              0,                           0,            1,  l_7], 
                           [              0,                           0,            0,    1]])
        T_W_7 = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ T_6_7
        jointPositions[7,0] = T_W_7[0,3]
        jointPositions[7,1] = T_W_7[1,3]
        jointPositions[7,2] = T_W_7[2,3]
        T0e = T_W_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ T_6_7 
        # Your code ends here


        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,0,0,0,0])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
