import numpy as np

from sensor.sensor import *
import os
import csv
import math
from pyquaternion import Quaternion
from enum import Enum

from abc import ABCMeta, abstractmethod


class IDistance:
    __metaclass__ = ABCMeta
    @abstractmethod
    def distance(self, q0: Quaternion, q1: Quaternion) -> float:
        raise NotImplemented


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
    Y = "Yaxis"
    Z = "Zaxis"


# initial setup
axis = Axis.X
sensor = Sensor.RELATIVE
date_list = {"2021-03-12", "2021-03-19", "2021-03-26", "2021-04-09", "2021-04-09"}

# base_path = "/mar 2021/agmundet/NDOF"
#base_path = f"C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet 3 - 180\\{sensor.value}"
#base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\april 2021\\agmundet 4\\{sensor.value}"
base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\april 2021\\agmundet 4"

filename = get_filename_by_date(os.path.join(base_path, sensor.value), f"{axis.value}\\{sensor.value}_{axis.value}", date_list)
if filename == "":
    print("Error, abort everything!")

with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')


# now `filename` contains the file to experiment
list_entries = list()
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        dic_entry = dict()

        n_exp = int(row[0])
        degree = int(row[1])
        yaw_pitch_roll = (float(row[2]), float(row[3]), float(row[4]))
        q = Quaternion(row[5], row[6], row[7], row[8])

        # add experiment number & degree & quaternion
        dic_entry["exp"] = n_exp
        dic_entry["degrees"] = degree
        dic_entry["q_w"], dic_entry["q_x"], dic_entry["q_y"], dic_entry["q_z"] = q.w, q.x, q.y, q.z

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
            #dic_entry["quat_dist"] = math.degrees(Quaternion.distance(q, q_theo)*2)
            dic_entry["quat_dist"] = math.degrees(Quaternion.distance(q, q_theo)*2)
            dic_entry["quat_sym_dist"] = math.degrees(Quaternion.sym_distance(q, q_theo)*2)
            dic_entry["quat_abs_dist"] = math.degrees(Quaternion.absolute_distance(q, q_theo)*2)

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
        list_entries.append(dic_entry)
        print()

q0 = Quaternion(w=list_entries[0]["q_w"], x=list_entries[0]["q_x"], y=list_entries[0]["q_y"], z=list_entries[0]["q_z"])
q1 = Quaternion(w=list_entries[1]["q_w"], x=list_entries[1]["q_x"],y=list_entries[1]["q_y"], z=list_entries[1]["q_z"])
q7 = Quaternion(w=list_entries[7]["q_w"], x=list_entries[7]["q_x"],y=list_entries[7]["q_y"], z=list_entries[7]["q_z"])
q8 = Quaternion(w=list_entries[8]["q_w"], x=list_entries[8]["q_x"],y=list_entries[8]["q_y"], z=list_entries[8]["q_z"])
q9 = Quaternion(w=list_entries[9]["q_w"], x=list_entries[9]["q_x"],y=list_entries[9]["q_y"], z=list_entries[9]["q_z"])

qt0 = Quaternion(axis=[0, 0, -1], degrees=0)
qt1 = Quaternion(axis=[0, 0, -1], degrees=10)
qt8 = Quaternion(axis=[0, 0, -1], degrees=80)

qt8a = Quaternion(axis=[0, 0, -1], degrees=80-9)
qt8b = Quaternion(axis=[0, 0, -1], degrees=80+9)
qt8a = Quaternion(axis=q0.axis, degrees=80-9)
qt8b = Quaternion(axis=q0.axis, degrees=80+9)
print(Quaternion.distance(qt8, qt8a),Quaternion.distance(qt8, qt8b))


def euler_to_quaternion(roll, pitch, yaw) -> Quaternion:
    """"Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention"""
    yaw = radians(yaw)
    pitch = radians(pitch)
    roll = radians(roll)
    return Quaternion(w=np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2),
                      x=np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2),
                      y=np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2),
                      z=np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2))


qt8a.yaw_pitch_roll

qt = Quaternion(axis=q0.axis, degrees=-(q0.degrees+10))
print(math.degrees(Quaternion.distance(q1, q0)*2))
print(math.degrees(Quaternion.distance(q1, qt)*2))

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]

# taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def to_quaternion(yaw, pitch, roll) -> Quaternion:
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Quaternion(w=w, x=x, y=y, z=z)



from scipy.spatial.transform import Rotation
def from_euler(x, y, z) -> Quaternion:
    rot = Rotation.from_euler('xyz', [x, y, z], degrees=True)
    return Quaternion(rot.as_quat())


import seaborn as sns
import os
import pandas as pd
import matplotlib.pyplot as plt
evaluating = "Z"
# to check the keys dic_entry.keys()
df = pd.DataFrame(list_entries, index=None)
ax = sns.boxplot(x="degrees", y="quat_dist", palette="Blues", data=df).set(title=f"{sensor.name.lower()} in axis {Axis.X}",
                                                                           ylabel=f'Error in {evaluating}')


# df = pd.DataFrame(list_distances,
#                   columns=["exp", "degrees", "qw", "qx", "qy", "qz", "Diff-X", "Diff-Y", "Diff-Z", "d1", "d2", "d3",
#                            "yaw", "pitch", "roll", "dist_quat", "dist_quat_sym", "dist_quat_abs"], index=None)


# ax = sns.boxplot(x="degrees", y="dist_quat_sym", palette="Blues", data=df).set(title=f"{name} in axis {axis[0]}", ylabel=f'Error in {evaluating}')

# ax = sns.boxplot(x="degrees", y="dist_quat_abs", palette="Oranges", data=df).set(title=f"{name} in axis {axis[0]}", ylabel=f'Error in {evaluating}')




plt.show()

def get_quat_theoretical(q_initial: Quaternion, axis: Axis, degrees: float):
    if axis == Axis.Z:
        qr = from_euler(degrees, 0, 0)
    elif axis == Axis.X:
        qr = from_euler(0, 0, degrees)
    elif axis == Axis.Y:
        qr = from_euler(0, degrees, 0)
    return qr * q_initial


list_distances = list()

# start to work with list_entries
for n_exp in range(1, 2):
    list_exp = [entry for entry in list_entries if entry["exp"] == n_exp]
    q_initial = Quaternion(list_exp[0]["q_w"], list_exp[0]["q_x"], list_exp[0]["q_y"], list_exp[0]["q_z"])
    list_exp = list_exp[1:]

    dict_distances = {}
    for entry in list_exp:
        if axis == Axis.Z:
            q_axis_rotated = from_euler(180 + entry["degrees"], 0, 0)
        elif axis == Axis.Y:
            q_axis_rotated = from_euler(entry["degrees"], 0, 0)
        elif axis == Axis.X:
            q_axis_rotated = from_euler(0, 0, 180 + entry["degrees"])
            q_axis_rotated = Quaternion(axis[0, 0, 1], degrees=180 + entry["degrees"])

        q_theo = q_axis_rotated * q_initial
        print(q_theo.degrees)
        q_current = Quaternion(entry["q_w"], entry["q_x"], entry["q_y"], entry["q_z"])
        #print(q_current.degrees)
        distance = Quaternion.distance(q_current, q_theo)
        #print(f"{q_current}, {q_theo}")
        #print(degrees(distance*2))


# create
n_exp = 1
result = [entry for entry in list_entries if entry["degrees"] == 20 and entry["exp"] == n_exp]
current = result[0]
q_current = Quaternion(current["q_w"], current["q_x"], current["q_y"], current["q_z"])

first = [entry for entry in list_entries if entry["degrees"] == 0 and entry["exp"] == n_exp]
first = first[0]

q_initial = Quaternion(first["q_w"], first["q_x"], first["q_y"], first["q_z"])
qr = from_euler(current["degrees"], 0, 0)
qt = qr * q_initial
print(180-degrees(Quaternion.sym_distance(q_current, qt)*2))
print(170-qt.degrees)
qt = q_initial * qr
print(degrees(Quaternion.absolute_distance(q_current, qt)*2))
print(170-qt.degrees)

index = 3
q_initial = Quaternion(first["q_w"], first["q_x"], first["q_y"], first["q_z"])
q10 = Quaternion(list_entries[index]["q_w"], list_entries[index]["q_x"], list_entries[index]["q_y"], list_entries[index]["q_z"])
q10_theo = q_initial*from_euler(float(list_entries[index]["degrees"]), 0, 0)
print(180-degrees(Quaternion.distance(q10_theo, q10)*2))
q10_theo = from_euler(float(list_entries[index]["degrees"]), 0, 0)*q_initial
print(180-degrees(Quaternion.distance(q10_theo, q10)*2))

from scipy.spatial import distance_matrix

# new approach (again)
def get_quat(list_values: list, pos: int) -> Quaternion:
    return Quaternion(w=list_entries[pos]["q_w"], x=list_entries[pos]["q_x"], y=list_entries[pos]["q_y"], z=list_entries[pos]["q_z"])

# https://medium.com/swlh/euclidean-distance-matrix-4c3e1378d87f
def EDM(A, B):
    p1 = np.sum(A**2, axis=1)[:, np.newaxis]
    p2 = np.sum(B**2, axis=1)
    p3 = -2 * np.dot(A, B.T)
    return np.round(np.sqrt(p1 + p2 + p3), 2)


#
t10 = get_quat(list_entries, 1)
degree_u = -5
degree_t = +5
q = t10

q_initial = Quaternion(0.769592, -0.000183, 0.004272, -0.63855)
q_initial = to_quaternion(0, 0, 0)

degree_u, degree_t = -50, +50
list_distances = list()
for yaw in range(degree_u, degree_t):
    for pitch in range(degree_u, degree_t):
        for roll in range(degree_u, degree_t):
            rot = to_quaternion(yaw, pitch, roll).rotation_matrix
            rot_curr = np.matmul(rot, q_initial.rotation_matrix)
            distance = np.linalg.norm(rot_curr)
            list_distances.append({"yaw": yaw, "pitch": pitch, "roll": roll, "distance": distance})

df = pd.DataFrame(list_distances, index=None)
#sns.lineplot(data=df, x="yaw", y="distance")
#sns.lineplot(data=df, x="pitch", y="distance")
sns.lineplot(data=df, x="roll", y="distance")

plt.show()

t1 = Quaternion(list_entries[0]["q_w"],list_entries[0]["q_x"], list_entries[0]["q_y"], list_entries[0]["q_z"])
t2 = Quaternion(list_entries[35]["q_w"],list_entries[35]["q_x"], list_entries[35]["q_y"], list_entries[35]["q_z"])
t2 = Quaternion(axis=t1.axis, degrees=270)
t2 = to_quaternion(yaw=10.0, pitch=0.0, roll=0.0)
t22 = euler_to_quaternion(yaw=10.0, pitch=0.0, roll=0.0)
print(degrees(Quaternion.distance(t1, t2)*2))
q1=Quaternion([0.0871557, 0, 0, 0.9961947 ])
q2=Quaternion([ 0, 0.0871557, 0, 0.9961947 ])
q3=Quaternion([ 0, 0, 0.0871557, 0.9961947 ])
print(degrees(Quaternion.distance(t1, q1)*2))
print(degrees(Quaternion.distance(t1, q2)*2))
print(degrees(Quaternion.distance(t1, q3)*2))
t23 = from_euler(10, 10, 0)
print(degrees(Quaternion.distance(t1, t23)*2))

