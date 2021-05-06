""" Works with mar2021/agmundet"""

from python.sensor.sensor import *
import os
import csv
import math
from pyquaternion import Quaternion

def rot_axis_angle_to_ypr(degrees: float, q: Quaternion, source: np.ndarray) -> tuple:
    """
    Function to rotate `source` an amount of `degrees` around the vector `v `
    :param degrees: a value in degrees
    :param q: axis vector where to rotate
    :param source: vector to be rotated
    :return: a tuple value representing yaw, pitch, roll
    """
    rotation_matrix = get_rotmat(radians(degrees), q[0], q[1], q[2])
    return rotation_matrix_to_ypr(np.matmul(rotation_matrix, source))

#folder = "python\\mar 2021\\agmundet"

axis = "Xaxis"
#sensor = "IMU"
sensor = "NDOF"
#base_path = "C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet\\NDOF"
#filename = f"{axis}\\Eje 1 (brazo largo)\\{sensor}_{axis}_2021-03-12.txt"
base_path = f"C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet 2\\{sensor}"
#filename = f"{axis}\\{sensor}_{axis}_2021-03-12.txt"
filename = f"{axis}\\{sensor}_{axis}_2021-03-19.txt"
filename = os.path.join(base_path, filename)
fcn_format_ypr = lambda x: f'({x[0]:.3f}, {x[1]:.3f}, {x[2]:.3f})'

list_rotations = list()
list_distances = list()
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        list_row = list()
        n_exp = int(row[0])
        degree = int(row[1])
        yaw_pitch_roll = (float(row[2]), float(row[3]), float(row[4]))
        q = Quaternion(row[5], row[6], row[7], row[8])

        # add experiment number & degree & quaternion
        list_row.append(n_exp)
        list_row.append(degree)
        list_row.extend(q)

        print(f"({degree}) degrees: original ypr: {yaw_pitch_roll}, quaternion: {q}")
        rot_mat_sensor = quat_to_rotmat_sensor(q.w, q.x, q.y, q.z)
        #print(rot_mat_sensor)
        #print(q.rotation_matrix)
        #print(q.get_axis())
        #print(q.radians)
        print(q.degrees)
        #print(q.angle)
        #rot_mat_sensor_ada = quat_to_rotmat_sensor_adafruit(q.w, q.x, q.y, q.z)
        #rot_mat_world = quat_to_rotmat_world(q)

        ypr1 = rotation_matrix_to_ypr(rot_mat_sensor)
        #ypr2 = quat_to_rotmat_sensor_adafruit(q.w, q.x, q.y, q.z)
        ypr2 = rotation_matrix_to_ypr(q.rotation_matrix)
        #ypr2 = rotation_matrix_to_ypr(rot_mat_world)


        print(f'ypr sensor: {fcn_format_ypr(ypr1)}  ypr ada: {fcn_format_ypr(ypr2)}')
        #print(f'ypr sensor: {fcn_format_ypr(ypr1)}')
        # print(f"quaternion2rot-0 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world1(q.w, q.x, q.y, q.z)))}")
        # print(f"quaternion2rot-1 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world2(q.w, q.x, q.y, q.z)))}")
        # print(f"quaternion2rot-2 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world3(q.w, q.x, q.y, q.z)))}")
        #get_rotmat(q.w, q.x, q.y, q.z)
        if degree == 0:
            rotmat0_sensor = rot_mat_sensor
            q0 = q
            # add 3 zeros as distances
            list_row.extend([0, 0, 0])
            #rotmat0_world = rot_mat_world
            #rotmat0_sensor_ada = rot_mat_sensor_ada
        else:
            theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 1], rotmat0_sensor)
            print(f"theo_sensor_ypr-0 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            ypr = rotation_matrix_to_ypr(q.rotation_matrix)
            if (degree == 100 or degree == 90)and axis == "Yaxis":
                diffX = 0
                diffY = 0
                diffZ = 0
                print(diffX, diffY, diffZ)
            else:
                diffX = theoretical_yaws_roll[0] - ypr[0]
                if abs(diffX) > 350:
                    diffX = 360 + diffX
                diffY = theoretical_yaws_roll[1] - ypr[1]
                if abs(diffY) > 180:
                    diffY = 180 - diffY
                diffZ = theoretical_yaws_roll[2] - ypr[2]
                if abs(diffZ) > 350:
                    diffZ = 360 + diffZ
            list_row.extend([diffX, diffY, diffZ])

            #theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 1], rotmat0_sensor)
            #print(f"theo_sensor_ypr-1 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            #theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 2], rotmat0_sensor)
            #print(f"theo_sensor_ypr-2 function: {fcn_format_ypr(theoretical_yaws_roll)}")

            print(Quaternion.distance(q0, q))
            print(Quaternion.absolute_distance(q0, q))
            print(Quaternion.sym_distance(q0, q))
            list_row.extend([Quaternion.distance(q0, q), Quaternion.absolute_distance(q0, q), Quaternion.sym_distance(q0, q)])
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_world[:, 0], rotmat0_world)
            # print(f"theo_world_ypr-0 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_world[:, 1], rotmat0_world)
            # print(f"theo_world_ypr-1 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_world[:, 2], rotmat0_world)
            # print(f"theo_world_ypr-2 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, [1, 0, 0], rotmat0)
            # print(fcn_format_ypr(theoretical_yaws_roll))
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, [0, 1, 0], rotmat0)
            # print(fcn_format_ypr(theoretical_yaws_roll))
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, [0, 0, 1], rotmat0)
            # print(fcn_format_ypr(theoretical_yaws_roll))
        list_distances.append(list_row)
        print()

import seaborn as sns
import os
import pandas as pd


name = "absolute" if sensor == "NDOF" else "relative"
evaluating = "Z"
df = pd.DataFrame(list_distances, columns=["exp", "degrees", "qw", "qx", "qy", "qz", "Diff-X", "Diff-Y", "Diff-Z", "d1", "d2", "d3"], index=None)

ax = sns.boxplot(x="degrees", y="d2", data=df).set(title=f"{name} in axis {axis[0]}",
                                                                   ylabel=f'Error in {evaluating}*')
