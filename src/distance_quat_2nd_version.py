""" Works with mar 2021 """
from python.sensor.sensor import *
import os
import csv
import math
from pyquaternion import Quaternion
from enum import Enum


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


def get_filename_by_date(root_path: str, prefix_path: str, dates_set: set) -> str:
    """
    Check all possible combination for files in the folders
    :param root_path:
    :param prefix_path:
    :param dates_set:
    :return:
    """
    for date in dates_set:
        full_path = f"{prefix_path}_{date}.txt"
        full_path = os.path.join(root_path, full_path)
        if os.path.exists(full_path):
            return full_path
    return ""


def fcn_format_ypr(x):
    return f"({x[0]:.3f}, {x[1]:.3f}, {x[2]:.3f})"


class Sensor(Enum):
    RELATIVE = "IMU"
    ABSOLUTE = "NDOF"


class Axis(Enum):
    X = "Xaxis"
    Y = "Xaxis"
    Z = "Xaxis"


# initial setup
axis = Axis.Z
sensor = Sensor.RELATIVE
date_list = {"2021-03-12", "2021-03-19", "2021-03-26", "2021-04-09"}

# base_path = "/mar 2021/agmundet/NDOF"
base_path = f"C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet 3 - 180\\{sensor.value}"

filename = get_filename_by_date(base_path, f"{axis.value}\\{sensor.value}_{axis.value}", date_list)
if filename == "":
    print("Error, abort everything!")

# now `filename` contains the file to experiment
list_rotations = list()
list_distances = list()
list_entries = list()
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        list_row = list()
        dic_entry = dict()

        n_exp = int(row[0])
        degree = int(row[1])
        yaw_pitch_roll = (float(row[2]), float(row[3]), float(row[4]))
        q = Quaternion(row[5], row[6], row[7], row[8])

        # add experiment number & degree & quaternion
        dic_entry["exp"] = n_exp
        dic_entry["degrees"] = degree
        dic_entry["q_w"] = q.w
        dic_entry["q_x"] = q.x
        dic_entry["q_y"] = q.y
        dic_entry["q_z"] = q.z
        list_row.append(n_exp)
        list_row.append(degree)
        list_row.extend(q)

        print(f"({degree}) degrees: original ypr: {yaw_pitch_roll}, quaternion: {q}")
        rot_mat_sensor = quat_to_rotmat_sensor(q.w, q.x, q.y, q.z)
        # print(rot_mat_sensor)
        # print(q.rotation_matrix)
        # print(q.get_axis())
        # print(q.radians)
        print(q.degrees)
        # print(q.angle)
        # rot_mat_sensor_ada = quat_to_rotmat_sensor_adafruit(q.w, q.x, q.y, q.z)
        # rot_mat_world = quat_to_rotmat_world(q)

        ypr1 = rotation_matrix_to_ypr(rot_mat_sensor)
        # ypr2 = quat_to_rotmat_sensor_adafruit(q.w, q.x, q.y, q.z)
        ypr2 = rotation_matrix_to_ypr(q.rotation_matrix)
        # ypr2 = rotation_matrix_to_ypr(rot_mat_world)

        print(f'ypr sensor: {fcn_format_ypr(ypr1)}  ypr ada: {fcn_format_ypr(ypr2)}')
        # print(f'ypr sensor: {fcn_format_ypr(ypr1)}')
        # print(f"quaternion2rot-0 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world1(q.w, q.x, q.y, q.z)))}")
        # print(f"quaternion2rot-1 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world2(q.w, q.x, q.y, q.z)))}")
        # print(f"quaternion2rot-2 function: {fcn_format_ypr(rotation_matrix_to_ypr(quaternion2rotmatrix_world3(q.w, q.x, q.y, q.z)))}")
        # get_rotmat(q.w, q.x, q.y, q.z)
        if degree == 0:
            rotmat0_sensor = rot_mat_sensor
            q0 = q
            # add 3 zeros as distances
            list_row.extend([0, 0, 0])
            dic_entry["Diff-X"] = 0.0
            dic_entry["Diff-Y"] = 0.0
            dic_entry["Diff-Z"] = 0.0
            # rotmat0_world = rot_mat_world
            # rotmat0_sensor_ada = rot_mat_sensor_ada
        else:
            # compute theoretical quaternion
            q_theo = Quaternion(axis=q0.axis, degrees=q0.degrees + degree)
            dist_quat0 = Quaternion.distance(q, q_theo)
            dist_quat1 = Quaternion.sym_distance(q, q_theo)
            dist_quat2 = Quaternion.absolute_distance(q, q_theo)
            dic_entry["quat_dist"] = Quaternion.distance(q, q_theo)
            dic_entry["quat_sym_dist"] = Quaternion.sym_distance(q, q_theo)
            dic_entry["quat_abs_dist"] = Quaternion.absolute_distance(q, q_theo)

            theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 1], rotmat0_sensor)
            print(f"theo_sensor_ypr-0 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            ypr = rotation_matrix_to_ypr(q.rotation_matrix)
            if (degree == 100 or degree == 90) and axis == "Yaxis":
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

            dic_entry["Diff-X"] = diffX
            dic_entry["Diff-Y"] = diffY
            dic_entry["Diff-Z"] = diffZ
            list_row.extend([diffX, diffY, diffZ])

            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 1], rotmat0_sensor)
            # print(f"theo_sensor_ypr-1 function: {fcn_format_ypr(theoretical_yaws_roll)}")
            # theoretical_yaws_roll = rot_axis_angle_to_ypr(degree, rotmat0_sensor[:, 2], rotmat0_sensor)
            # print(f"theo_sensor_ypr-2 function: {fcn_format_ypr(theoretical_yaws_roll)}")

            # print(Quaternion.distance(q0, q))
            # print(Quaternion.absolute_distance(q0, q))
            # print(Quaternion.sym_distance(q0, q))
            dic_entry["d1"] = Quaternion.distance(q0, q)
            dic_entry["d2"] = Quaternion.absolute_distance(q0, q)
            dic_entry["d3"] = Quaternion.sym_distance(q0, q)
            list_row.extend(
                [Quaternion.distance(q0, q), Quaternion.absolute_distance(q0, q), Quaternion.sym_distance(q0, q)])

            # d = abs((q - q0).degrees)
            d = Quaternion.distance(q, q0)
            ypr_ori = np.array(rotation_matrix_to_ypr(rotmat0_sensor))
            ypr_curr = np.array(rotation_matrix_to_ypr(q.rotation_matrix))

            ypr = np.array(ypr_curr) - np.array(ypr_ori)

            if axis == "Xaxis":
                ypr[2] = degree + ypr[2]
            elif axis == "Yaxis":
                if degree >= 90:
                    if degree > 90:
                        ypr[0] = 180 - ypr_curr[2] - ypr_ori[2]
                        ypr[1] = 180 - degree + ypr[1]
                        ypr[2] = 180 + ypr_curr[0] - ypr_ori[0]
                    else:
                        ypr[0] = 0
                        ypr[1] = 0
                        ypr[2] = 0
                else:
                    ypr[1] = degree - abs(ypr[1])
            elif axis == "Zaxis":
                ypr[0] = 360 - (degree + ypr[0] % 360)

            # just for IMU
            if degree > 160 and ypr[2] > 150:
                if ypr[2] > 330:
                    ypr[2] = 360 - ypr[2]
                else:
                    ypr[2] = 180 - ypr[2]

            print(f"original: {fcn_format_ypr(ypr_ori)}")
            print(f"distance: {fcn_format_ypr(ypr)}")
            dic_entry["yaw"] = ypr[0]
            dic_entry["pitch"] = ypr[1]
            dic_entry["roll"] = ypr[2]
            list_row.extend(ypr)
            list_row.append(dist_quat0)
            list_row.append(dist_quat1)
            list_row.append(dist_quat2)
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
        list_entries.append(dic_entry)
        print()

import seaborn as sns
import os
import pandas as pd
import matplotlib.pyplot as plt
evaluating = "Z"
df = pd.DataFrame(list_distances,
                  columns=["exp", "degrees", "qw", "qx", "qy", "qz", "Diff-X", "Diff-Y", "Diff-Z", "d1", "d2", "d3",
                           "yaw", "pitch", "roll", "dist_quat", "dist_quat_sym", "dist_quat_abs"], index=None)
ax = sns.boxplot(x="degrees", y="dist_quat_abs", palette="Blues", data=df).set(title=f"{sensor.name.lower()} in axis {Axis.X}",
                                                                           ylabel=f'Error in {evaluating}')

# ax = sns.boxplot(x="degrees", y="dist_quat_sym", palette="Blues", data=df).set(title=f"{name} in axis {axis[0]}", ylabel=f'Error in {evaluating}')

# ax = sns.boxplot(x="degrees", y="dist_quat_abs", palette="Oranges", data=df).set(title=f"{name} in axis {axis[0]}", ylabel=f'Error in {evaluating}')

plt.show()
