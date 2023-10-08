from lib.calculateFK import FK
from core.interfaces import ArmController
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as tri
from matplotlib import cm
from matplotlib.ticker import MaxNLocator

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]
num_samples = 6  # Adjust as needed for desired resolution
joint_ranges = [np.linspace(joint['lower'], joint['upper'], num_samples) for joint in limits]
end_effector_positions = []
for q1 in joint_ranges[0]:
    for q2 in joint_ranges[1]:
        for q3 in joint_ranges[2]:
            for q4 in joint_ranges[3]:
                for q5 in joint_ranges[4]:
                    for q6 in joint_ranges[5]:
                        for q7 in joint_ranges[6]:
                            joint_angles = [q1, q2, q3, q4, q5, q6, q7]
                            end_effector_position, T = fk.forward(joint_angles)
                            end_effector_positions.append([end_effector_position[7][0],end_effector_position[7][1],end_effector_position[7][2]])


x_coords, y_coords, z_coords = zip(*end_effector_positions)
# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



ax.scatter(x_coords, y_coords, z_coords, c='g', marker='o', alpha = 0.01)
plt.show()
