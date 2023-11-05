import numpy as np 
from lib.calcJacobian import calcJacobian

def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    np_v = np.isnan(v_in)
    np_w = np.isnan(omega_in)
    # print("v_in: ", v_in)
    # print("omega_in: ", omega_in)
    v_in[np_v] = 0
    omega_in[np_w] = 0

    dq = np.zeros((1, 7))
    # print("v_in: ", v_in)
    # print("omega_in: ", omega_in)


    J = calcJacobian(q_in)
    # print("J: ", J)
    J_pi = np.linalg.lstsq(J, np.identity(6), rcond=None)[0]
    # print("J_pi: ", J_pi)
    epsilon = np.zeros((6, 1))
    # print("epsilon shape: ", epsilon.shape)
    # print("vin shape: ", v_in.shape)
    epsilon[0,0] = v_in[0, 0]
    epsilon[1,0] = v_in[0, 1]
    epsilon[2,0] = v_in[0, 2]
    epsilon[3,0] = omega_in[0]
    epsilon[4,0] = omega_in[1]
    epsilon[5,0] = omega_in[2]
    epsilon.reshape(6,)
    # print("epsilon: ", epsilon)
    dq = J_pi @ epsilon
    # print("dq: ", dq)
    threshold = .69

    mask = np.abs(dq) > threshold
    print(dq)
    print(mask)

    if True in mask:
        scaling_factor = threshold/dq.max()
        dq = dq * scaling_factor
    print(dq)
    return dq.flatten()

