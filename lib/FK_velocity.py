import numpy as np 
from lib.calcJacobian import calcJacobian
from math import pi

def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE
    J = calcJacobian(q_in)

    velocity = np.zeros((6, 1))
    velocity = J @ np.transpose(dq)

    print("velocity: ", velocity)

    return velocity

if __name__ == "__main__":


    # matches figure in the handout
    q = np.array([ 0,    0,        0,  -pi/2,     0,  pi/2, pi/4 ])
    dq = np.array([ 0,    0,        0,  .5,     0,  0, 0 ])
    FK_velocity(q, dq)

    
