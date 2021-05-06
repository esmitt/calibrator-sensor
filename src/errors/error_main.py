import math

from errors.reference import compute_reference
from errors.converter import to_quaternion
from sensor.sensor import quat_to_rotmat_sensor, rotation_matrix_to_ypr
from sensor.sensor import get_rotmat
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import csv
from sensor import sensor
from pyquaternion import Quaternion
from typing import List, Dict, Tuple, Any, Optional
from enum import Enum
from math import radians

sns.set() # Setting seaborn as default style even if use only matplotlib
sns.set_theme(style="darkgrid")

import matplotlib

matplotlib.use('TkAgg' )


class Axis(Enum):
    X = "Xaxis"
    Y = "Yaxis"
    Z = "Zaxis"

class Sensor(Enum):
    RELATIVE = "IMU"
    ABSOLUTE = "NDOF"


def read_csv(filename: str) -> List[Dict]:
    list_entries = []
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            dic_entry = {}
            yaw_pitch_roll = (float(row[2]), float(row[3]), float(row[4]))
            q = Quaternion(row[5], row[6], row[7], row[8])

            # add experiment number & degree & quaternion
            dic_entry["exp"] = int(row[0])
            dic_entry["degrees"] = int(row[1])
            dic_entry["ypr"] = (float(row[2]), float(row[3]), float(row[4]))
            # dic_entry["yaw"] = float(row[2])
            # dic_entry["pitch"] = float(row[3])
            # dic_entry["roll"] = float(row[4])
            dic_entry["q"] = q
            list_entries.append(dic_entry)
    return list_entries


def difference_float(v1: Any, v2: Any) -> Any:
    return max(v1, v2) - min(v1, v2)


def difference_tuple(v1: Tuple, v2: Tuple) -> Tuple:
    return difference_float(v1[0], v2[0]), difference_float(v1[1], v2[1]), difference_float(v1[2], v2[2])


# for 180 degree number_degrees should be 18, for 360 should be 36
def compute_distances_ypr_sensor(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    list_distances: List = []

    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        ypr_initial = list_experiment[0]['ypr']
        list_experiment[0]['distance_yaw'] = 0
        list_experiment[0]['distance_pitch'] = 0
        list_experiment[0]['distance_roll'] = 0
        for degree in range(1, number_degrees):
            #distance = np.array(list_experiment[degree]['ypr']) - np.array(ypr_initial)
            distance = difference_tuple(list_experiment[degree]['ypr'], ypr_initial)
            #distance = [d % 360 if d < 0 else d for d in distance]

            # check the swap
            list_experiment[degree]['distance_yaw'] = distance[2]
            list_experiment[degree]['distance_pitch'] = distance[1]
            list_experiment[degree]['distance_roll'] = distance[0]

            if axis == Axis.X:
                #list_experiment[degree]['distance_yaw'] = abs(list_experiment[degree]["degrees"] - distance[2]) % 360
                list_experiment[degree]['distance_yaw'] = difference_float(list_experiment[degree]["degrees"], distance[2]) % 360
            elif axis == Axis.Y:
                #list_experiment[degree]['distance_pitch'] = abs(list_experiment[degree]["degrees"] - distance[1]) % 360
                list_experiment[degree]['distance_pitch'] = difference_float(list_experiment[degree]["degrees"], distance[1]) % 360
            elif axis == Axis.Z:
                #list_experiment[degree]['distance_roll'] = abs(list_experiment[degree]["degrees"] - distance[0]) % 360
                list_experiment[degree]['distance_roll'] = difference_float(list_experiment[degree]["degrees"], distance[0]) % 360

            list_experiment[degree]['distance_yaw'] = abs(list_experiment[degree]['distance_yaw'])
            list_experiment[degree]['distance_pitch'] = abs(list_experiment[degree]['distance_pitch'])
            list_experiment[degree]['distance_roll'] = abs(list_experiment[degree]['distance_roll'])

        list_distances.extend(list_experiment)
    return list_distances


def compute_distances_ypr_quat(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    def quaternion_to_ypr(q: Quaternion) -> Tuple:
        rot_mat_sensor0 = quat_to_rotmat_sensor(q.w, q.x, q.y, q.z)
        return rotation_matrix_to_ypr(rot_mat_sensor0)

    list_distances = []

    for index_exp in range(1, number_experiments + 1):
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]  # <-- current extract experiment

        ypr_initial = quaternion_to_ypr(list_experiment[0]['q'])
        list_experiment[0]['distance_yaw'], list_experiment[0]['distance_pitch'], list_experiment[0]['distance_roll'] = 0, 0, 0
        for degree in range(1, number_degrees):
            ypr = quaternion_to_ypr(list_experiment[degree]['q'])
            distance = difference_tuple(ypr_initial, ypr)   # <-- difference_tuple(x, y): return max(x, y) - min(x, y)

            list_experiment[degree]['distance_yaw'] = abs(distance[2])
            list_experiment[degree]['distance_pitch'] = abs(distance[1])
            list_experiment[degree]['distance_roll'] = abs(distance[0])

            if axis == Axis.X:
                list_experiment[degree]['distance_yaw'] = abs(difference_float(list_experiment[degree]["degrees"], abs(distance[2])))
            elif axis == Axis.Y:
                list_experiment[degree]['distance_pitch'] = abs(difference_float(list_experiment[degree]["degrees"], abs(distance[1])))
            elif axis == Axis.Z:
                list_experiment[degree]['distance_roll'] = abs(difference_float(list_experiment[degree]["degrees"], abs(distance[0])))

        list_distances.extend(list_experiment)
    return list_distances


# error between q - q0
# def compute_distances_quat(list_sensor: List, number_experiments: Optional[int] = 10,
#                            number_degrees: Optional[int] = 18) -> List:
#
#     #number_experiments: int = 5
#     #number_degrees: int = 36
#     list_distances = []
#     for index_exp in range(1, number_experiments + 1):
#         # extract experiment according to the index
#         list_experiment = [item for item in list_sensor if item['exp'] == index_exp]
#
#         q_initial = list_experiment[0]['q']
#         list_experiment[0]['distance'] = 0
#         for degree in range(1, number_degrees):
#             q = list_experiment[degree]['q']
#             distance = Quaternion.distance(q, q_initial)
#             list_experiment[degree]['distance'] = distance
#
#         list_distances.extend(list_experiment)
#     return list_distances

def compute_distances_quat(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    axis_rot = [0, 0, 0]
    if axis == Axis.X:
        axis_rot = [1, 0, 0]
    if axis == Axis.Y:
        axis_rot = [0, 1, 0]
    if axis == Axis.Z:
        axis_rot = [0, 0, 1]

    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]    # <-- contains a experiment

        q_initial = list_experiment[0]['q']     # <-- capture from sensor, the initial position
        list_experiment[0]['distance'] = 0
        for index in range(1, number_degrees):
            degree = list_experiment[index]["degrees"]
            q_theo = Quaternion(axis=axis_rot + q_initial.axis, degrees=q_initial.degrees + degree)
            q = list_experiment[index]['q']     # <-- this is the capture from sensor
            distance = Quaternion.distance(q_theo, q)   # <-- this is distance computation
            list_experiment[index]['distance'] = distance

        list_experiment.pop(0)  # remove when degrees == 0
        list_distances.extend(list_experiment)
    return list_distances

def compute_distances_quat_robot(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    axis_rot = [0, 0, 0]
    if axis == Axis.X:
        axis_rot = [1, 0, 0]
    if axis == Axis.Y:
        axis_rot = [0, 1, 0]
    if axis == Axis.Z:
        axis_rot = [0, 0, 1]

    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]    # <-- contains a experiment

        q_initial = list_experiment[0]['q']     # <-- capture from sensor, the initial position
        list_experiment[0]['distance'] = 0
        for index in range(1, number_degrees):
            degree = list_experiment[index]["degrees"]
            #q_theo = Quaternion(axis=axis_rot, degrees=degree)
            q_theo = Quaternion(axis=q_initial.axis, degrees=degree+q_initial.degrees)
            q = list_experiment[index]['q']     # <-- this is the capture from sensor

            distance = Quaternion.distance(q_theo, q)   # <-- this is distance computation
            list_experiment[index]['distance'] = distance
            distance = Quaternion.sym_distance(q_theo, q)  # <-- this is distance computation
            list_experiment[index]['sym_distance'] = distance
            distance = Quaternion.absolute_distance(q_theo, q)  # <-- this is distance computation
            list_experiment[index]['absolute_distance'] = distance

        list_experiment.pop(0)  # remove when degrees == 0
        list_distances.extend(list_experiment)
    return list_distances


def compute_distances_quat_as_theoretical(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]    # <-- contains a experiment

        q_initial = list_experiment[0]['q']     # <-- capture from sensor, the initial position
        yaw_initial, pitch_initial, roll_initial = rotation_matrix_to_ypr(q_initial.rotation_matrix)
        list_experiment[0]['distance'] = 0
        for index in range(1, number_degrees):
            degree = list_experiment[index]["degrees"]
            q_theo = Quaternion()
            if axis == Axis.Z:
                q_theo = to_quaternion(-degree + yaw_initial, pitch_initial, roll_initial)
              #  q_theo = to_quaternion(-degree + 10 + yaw_initial, pitch_initial, roll_initial) #cogiend el segundo como inicial
            if axis == Axis.Y:
                q_theo = to_quaternion(yaw_initial, -degree + pitch_initial, roll_initial)
            if axis == Axis.X:
                q_theo = to_quaternion(yaw_initial, pitch_initial, -degree + roll_initial)
            q = list_experiment[index]['q']     # <-- this is the capture from sensor
           # distance = Quaternion.absolute_distance(q_theo, q) *180/np.pi  # <-- this is distance computation
            distance1 = Quaternion.distance(q_theo, q)*180/np.pi
            distance2 = Quaternion.distance(q_theo, -q) * 180 / np.pi
            distance=min(distance1,distance2)
            # distance=distance*2 #to interpret as error only in the rottion angle
            list_experiment[index]['distance'] = distance

        list_experiment.pop(0)  # remove when degrees == 0
        list_distances.extend(list_experiment)
    return list_distances


def compute_distances_quat_approach1(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]    # <-- contains a experiment

        q_initial = list_experiment[0]['q']     # <-- capture from sensor, the initial position
        yaw_initial, pitch_initial, roll_initial = rotation_matrix_to_ypr(q_initial.rotation_matrix)
        list_experiment[0]['distance'] = 0
        for index in range(0, number_degrees):
            degree = list_experiment[index]["degrees"]
            q_theo = Quaternion()
            if axis == Axis.Z:
                q_theo = to_quaternion(-degree + yaw_initial, pitch_initial, roll_initial)
              #  q_theo = to_quaternion(-degree + 10 + yaw_initial, pitch_initial, roll_initial) #cogiend el segundo como inicial
            if axis == Axis.Y:
                q_theo = to_quaternion(yaw_initial, -degree + pitch_initial, roll_initial)
            if axis == Axis.X:
                q_theo = to_quaternion(yaw_initial, pitch_initial, -degree + roll_initial)
            q = list_experiment[index]['q']     # <-- this is the capture from sensor
           # distance = Quaternion.absolute_distance(q_theo, q) *180/np.pi  # <-- this is distance computation
            distance = min(math.degrees(Quaternion.distance(q_theo, q)), math.degrees(Quaternion.distance(q_theo, -q)))
            # distance=distance*2 #to interpret as error only in the rottion angle
            list_experiment[index]['distance_ap4'] = distance

            # approach 1
            rot = np.matmul(q.rotation_matrix, np.linalg.inv(q_initial.rotation_matrix))
            if axis == Axis.Z:
                q_theo = get_rotmat(-degree, 1, 0, 0)
            if axis == Axis.Y:
                q_theo = get_rotmat(-degree, 0, 1, 0)
            if axis == Axis.X:
                q_theo = get_rotmat(-degree, 0, 0, 1)
            distance = np.linalg.norm(rot - q.rotation_matrix)
            list_experiment[index]['distance_ap1'] = distance
            # approach 2
            if axis == Axis.Z:
                q_theo = Quaternion(axis=[0, 0, 1], degrees=q_initial.degrees - degree)
            if axis == Axis.Y:
                q_theo = Quaternion(axis=[0, 1, 0], degrees=q_initial.degrees - degree)
            if axis == Axis.X:
                q_theo = Quaternion(axis=[1, 0, 0], degrees=q_initial.degrees - degree)
            distance = min(math.degrees(Quaternion.distance(q_theo, q)), math.degrees(Quaternion.distance(q_theo, -q)))
            list_experiment[index]['distance_ap2'] = distance
            # approach 3
            if axis == Axis.Z:
                q_theo = to_quaternion(-degree, pitch_initial, roll_initial)
              #  q_theo = to_quaternion(-degree + 10 + yaw_initial, pitch_initial, roll_initial) #cogiend el segundo como inicial
            if axis == Axis.Y:
                q_theo = to_quaternion(yaw_initial, -degree, roll_initial)
            if axis == Axis.X:
                q_theo = to_quaternion(yaw_initial, pitch_initial, -degree)
            distance = min(math.degrees(Quaternion.distance(q_theo, q)), math.degrees(Quaternion.distance(q_theo, -q)))
            list_experiment[index]['distance_ap3'] = distance
            # approach 5
            q_theo = Quaternion(axis=q_initial.axis, degrees=-degree)
            distance = min(math.degrees(Quaternion.distance(q_theo, q)), math.degrees(Quaternion.distance(q_theo, -q)))
            list_experiment[index]['distance_ap5'] = distance

            #####################
            if axis == Axis.Z:
                q_theo = Quaternion(axis=[0, 0, 1], degrees=-degree)
            if axis == Axis.Y:
                q_theo = Quaternion(axis=[0, 1, 0], degrees=-degree)
            if axis == Axis.X:
                q_theo = Quaternion(axis=[1, 0, 0], degrees=-degree)

            # approach paper 1, distancia quaternion entre q_theo y q*inv(q0).
            q_temp = q * q_initial.inverse
            list_experiment[index]['distance1_paper'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)),
                                                              math.degrees(Quaternion.distance(q_theo, -q_temp)))

            # approach paper 2, distancia quaternion entre q_theo con el quaternion de la matriz rotación R_q*inv(R_q0), donde inv(R_q0) és la inversa de la matriz q0
            q_temp = Quaternion(matrix=np.matmul(q.rotation_matrix, np.linalg.inv(q_initial.rotation_matrix)))
            list_experiment[index]['distance2_paper'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)),
                                                              math.degrees(Quaternion.distance(q_theo, -q_temp)))

            # approach 3 norma L2 de Rq_theo-R_q*inv(R_q0)
            r_temp = q_theo.rotation_matrix - np.matmul(q.rotation_matrix, np.linalg.inv(q_initial.rotation_matrix))
            list_experiment[index]['distance3_paper'] = np.linalg.norm(r_temp)

            # approach solicitado por debora (05/05/2021) distance 1
            q_temp = q_initial.inverse * q
            q_temp.degrees
            list_experiment[index]['distance1_newpaper'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)),
                                                            math.degrees(Quaternion.distance(q_theo, -q_temp)))
            # approach solicitado por debora (05/05/2021) distance 2
            q_temp = Quaternion(matrix=np.matmul(np.linalg.inv(q_initial.rotation_matrix), q.rotation_matrix))
            list_experiment[index]['distance2_newpaper'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)),
                                                               math.degrees(Quaternion.distance(q_theo, -q_temp)))
            # approach solicitado por debora (05/05/2021) distance 2
            r_temp = q_theo.rotation_matrix - np.matmul(np.linalg.inv(q_initial.rotation_matrix), q.rotation_matrix)
            list_experiment[index]['distance3_newpaper'] = np.linalg.norm(r_temp)

            # q_temp = q*q_initial.inverse
            # list_experiment[index]['distance_paper1_1'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)), math.degrees(Quaternion.distance(q_theo, -q_temp)))
            # if axis == Axis.Z:
            #     q_theo = Quaternion(axis=[0, 0, 1], degrees=degree)
            # if axis == Axis.Y:
            #     q_theo = Quaternion(axis=[0, 1, 0], degrees=degree)
            # if axis == Axis.X:
            #     q_theo = Quaternion(axis=[1, 0, 0], degrees=degree)
            #
            # q_temp = Quaternion(matrix=np.matmul(np.linalg.inv(q_initial.rotation_matrix), q.rotation_matrix))
            # list_experiment[index]['distance_paper1_2'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)), math.degrees(Quaternion.distance(q_theo, -q_temp)))
            # q_temp = q*q_initial.inverse
            # list_experiment[index]['distance_paper1_3'] = min(math.degrees(Quaternion.distance(q_theo, q_temp)), math.degrees(Quaternion.distance(q_theo, -q_temp)))

            # approach paper 2

        list_experiment.pop(0)  # remove when degrees == 0
        list_distances.extend(list_experiment)
    return list_distances


def compute_distances_ypr_rot(list_sensor: List, axis: Axis, number_experiments: int, number_degrees: int) -> List:
    list_distances: List = []

    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        ypr_initial = list_experiment[0]['ypr']
        list_experiment[0]['distance_yaw'] = 0
        list_experiment[0]['distance_pitch'] = 0
        list_experiment[0]['distance_roll'] = 0
        r_initial = list_experiment[0]['q'].rotation_matrix
        for degree in range(1, number_degrees):
            #distance = np.array(list_experiment[degree]['ypr']) - np.array(ypr_initial)
            r_current = list_experiment[degree]['q'].rotation_matrix
            ypr_initial = rotation_matrix_to_ypr(r_initial)
            ypr_current = rotation_matrix_to_ypr(r_current)
            distance = difference_tuple(ypr_current, ypr_initial)

            # check the swap
            list_experiment[degree]['distance_yaw'] = distance[2]
            list_experiment[degree]['distance_pitch'] = distance[1]
            list_experiment[degree]['distance_roll'] = distance[0]

            if axis == Axis.X:
                list_experiment[degree]['distance_yaw'] = difference_float(list_experiment[degree]["degrees"], distance[2]) % 360
            elif axis == Axis.Y:
                list_experiment[degree]['distance_pitch'] = difference_float(list_experiment[degree]["degrees"], distance[1]) % 360
            elif axis == Axis.Z:
                list_experiment[degree]['distance_roll'] = difference_float(list_experiment[degree]["degrees"], distance[0]) % 360

            list_experiment[degree]['distance_yaw'] = abs(list_experiment[degree]['distance_yaw'])
            list_experiment[degree]['distance_pitch'] = abs(list_experiment[degree]['distance_pitch'])
            list_experiment[degree]['distance_roll'] = abs(list_experiment[degree]['distance_roll'])

        list_distances.extend(list_experiment)
    return list_distances


def compute_distances_own(list_sensor: List, axis :Axis, number_experiments: Optional[int] = 10,
                           number_degrees: Optional[int] = 18) -> List:
    list_distances = []
    axis_rot = [0]*3
    if axis == Axis.X:
        axis_rot = [1, 0, 0]
    if axis == Axis.Y:
        axis_rot = [0, 1, 0]
    if axis == Axis.Z:
        axis_rot = [0, 0, 1]

    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        q_initial = list_experiment[0]['q']
        list_experiment[0]['distance'] = 0
        for degree in range(1, number_degrees):
            sensor.get_rotmat(math.radians(list_experiment[degree]), axis_rot[0], axis_rot[1], axis_rot[2])
            r = q_initial.rotation_matrix

    # q = list_distances[1]["q"]
    # r = sensor.quat_to_rotmat_sensor(q.w, q.x, q.y, q.z)
    # rr = sensor.get_rotmat(0, 1, 0, 0)
    return list_distances

# error between q - qtheo
def compute_distances_quat_theo(list_sensor: List) -> List:
    number_experiments: int = 5
    number_degrees: int = 36
    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        q_initial = list_experiment[0]['q']
        list_experiment[0]['distance'] = 0
        for degree in range(1, number_degrees):
            q = list_experiment[degree]['q']
            q_theoretical = Quaternion(axis=q_initial.axis, degrees=list_experiment[degree]["degrees"] + q_initial.degrees)
            distance = Quaternion.distance(q, q_theoretical)
            list_experiment[degree]['distance'] = distance

        list_distances.extend(list_experiment)
    return list_distances


"""Contains the key key_to_plot in axis Y vs. `degrees` in axis X"""
def draw_box_plot(list_entry: List, key_to_plot: str, title: str, y_text:str):
    df_list = pd.DataFrame(list_entry, index=None)
    sns.set_context("paper", rc={"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 5})
    sret = sns.boxplot(x='degrees', y=f'{key_to_plot}', palette="Blues", data=df_list, showfliers=False)
    sret.set(title=title, ylabel=y_text)
    #, showfliers=False
    #plt.legend(title='Team', fontsize='2', title_fontsize='12')
    #plt.show()
    return sret.get_figure()


def load_reference():
    """Draw the reference code"""
    filename = "D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\reference.csv"
    df_reference = pd.read_csv(filename, delimiter=";")

    sns.boxplot(x='yaw', y="distance", palette="Blues", data=df_reference).set(title="Distance quaternion", ylabel="yaw")
    sns.boxplot(x='pitch', y="distance", palette="Blues", data=df_reference).set(title="Distance quaternion", ylabel="pitch")
    sns.boxplot(x='roll', y="distance", palette="Blues", data=df_reference).set(title="Distance quaternion", ylabel="roll")

    fig, axs = plt.subplots(ncols=3, figsize=(20, 10))
    sns.boxplot(x='yaw', y='distance', data=df_reference, ax=axs[0])
    sns.boxplot(x='pitch', y='distance', data=df_reference, ax=axs[1])
    sns.boxplot(x='roll', y='distance', data=df_reference, ax=axs[2])
    plt.show()
    # df_reference = compute_reference(0, 0, 0)
    # df_reference.head()
    # df_reference.to_csv('reference.csv', sep=';', index=False)
    # fig, axs = plt.subplots(ncols=3, figsize=(20, 10))
    # sns.pointplot(x='yaw', y='distance', data=df_reference, ax=axs[0])
    # sns.lineplot(x='pitch', y='distance', data=df_reference, ax=axs[1])
    # sns.boxplot(x='roll', y='distance', data=df_reference, ax=axs[2])
    # plt.show()
    # ax = sns.boxplot(x="degrees", y="dist_quat_abs", palette="Blues", data=df).set(title=f"{sensor.name.lower()} in axis {Axis.X}",
    #                                                                            ylabel=f'Error in {evaluating}')

def saveFig(database):
    experiments = list(database.keys())


    for experiment in experiments:
        if experiment == 180:
            number_experiments = 10
            number_degrees = 19
        elif experiment == 360:
            number_experiments = 5
            number_degrees = 37
        base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{experiment}"
        output_base = "D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python"

        for sensor in [Sensor.RELATIVE, Sensor.ABSOLUTE]:
            for axis in [Axis.X, Axis.Y, Axis.Z]:
                # load the proper file
                file_path = os.path.join(base_path, sensor.value)
                file_path = os.path.join(file_path, axis.value)
                filename = os.path.join(file_path, database[experiment][sensor][axis])
                print(filename)
                list_file = read_csv(filename)
                list_distances = compute_distances_quat_approach1(list_file, axis, number_experiments, number_degrees)
                plt.figure()
                filename = f"{axis.value}-{sensor.name.lower()}-{experiment}.png"
                output_path = os.path.join(output_base, "approach 1")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance_ap1", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "approach 2")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance_ap2", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "approach 3")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance_ap3", title=f"Error axis {axis.name}",
                          y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "approach 4")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance_ap4", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "approach 5")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance_ap5", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                #paper 1
                output_path = os.path.join(output_base, "distance 1")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance1_paper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "distance 2")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance2_paper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "distance 3")
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance3_paper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "distance 1(backward)")
                if not os.path.exists(output_path):
                    os.mkdir(output_path)
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance1_newpaper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "distance 2(backward)")
                if not os.path.exists(output_path):
                    os.mkdir(output_path)
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance2_newpaper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)
                plt.close('all')
                output_path = os.path.join(output_base, "distance 3(backward)")
                if not os.path.exists(output_path):
                    os.mkdir(output_path)
                output_path = os.path.join(output_path, filename)
                draw_box_plot(list_distances, f"distance3_newpaper", title=f"Error axis {axis.name}",
                              y_text=f"distance in quaternion").savefig(output_path)

    plt.close('all')
    # experiment = 360
    # number_experiments = 5
    # number_degrees = 37
    #
    # base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{experiment}"
    # for sensor in [Sensor.RELATIVE, Sensor.ABSOLUTE]:
    #     for axis in [Axis.X, Axis.Y, Axis.Z]:
    #         # load the proper file
    #         file_path = os.path.join(base_path, sensor.value)
    #         file_path = os.path.join(file_path, axis.value)
    #         filename = os.path.join(file_path, database[experiment][sensor][axis])
    #         print(filename)
    #         list_file = read_csv(filename)
    #         list_distances = compute_distances_quat_approach1(list_file, axis, number_experiments, number_degrees)
    #         plt.figure()
    #         filename = f"{axis.value}-{sensor.name.lower()}-{experiment}.png"
    #         output_path = os.path.join(output_base, "approach 1")
    #         output_path = os.path.join(output_path, filename)
    #         draw_box_plot(list_distances, f"distance_ap1", title=f"Error axis {axis.name}",
    #                       y_text=f"distance in quaternion").savefig(output_path)
    #         plt.close('all')
    #         output_path = os.path.join(output_base, "approach 2")
    #         output_path = os.path.join(output_path, filename)
    #         draw_box_plot(list_distances, f"distance_ap2", title=f"Error axis {axis.name}",
    #                       y_text=f"distance in quaternion").savefig(output_path)
    #         plt.close('all')
    #         output_path = os.path.join(output_base, "approach 3")
    #         output_path = os.path.join(output_path, filename)
    #         draw_box_plot(list_distances, f"distance_ap3", title=f"Error axis {axis.name}",
    #                       y_text=f"distance in quaternion").savefig(output_path)
    #         plt.close('all')
    #         output_path = os.path.join(output_base, "approach 4")
    #         output_path = os.path.join(output_path, filename)
    #         draw_box_plot(list_distances, f"distance_ap4", title=f"Error axis {axis.name}",
    #                       y_text=f"distance in quaternion").savefig(output_path)
    #         plt.close('all')
    #         output_path = os.path.join(output_base, "approach 5")
    #         output_path = os.path.join(output_path, filename)
    #         draw_box_plot(list_distances, f"distance_ap5", title=f"Error axis {axis.name}",
    #                       y_text=f"distance in quaternion").savefig(output_path)
    #         plt.close('all')
    # plt.close('all')

# experiment to demostrate that are different:
# q=Quaternion(axis=(0, 1, 0), degrees=90)
# print(q)
# print(q.axis)
if __name__ == "__main__":
    database = {
        360: {
            Sensor.RELATIVE: {
                Axis.X: "IMU_Xaxis_2021-04-09.txt",
                Axis.Y: "IMU_Yaxis_2021-04-09.txt",
                Axis.Z: "IMU_Zaxis_2021-03-26.txt"
            },
            Sensor.ABSOLUTE: {
                Axis.X: "NDOF_Xaxis_2021-04-09.txt",
                Axis.Y: "NDOF_Yaxis_2021-04-09.txt",
                Axis.Z: "NDOF_Zaxis_2021-04-09.txt"
            },
        },
        # 180: {
        #     Sensor.RELATIVE: {
        #         Axis.X: "IMU_Xaxis_2021-03-26.txt",
        #         Axis.Y: "IMU_Yaxis_2021-03-26.txt",
        #         Axis.Z: "IMU_Zaxis_2021-03-26.txt"
        #     },
        #     Sensor.ABSOLUTE: {
        #         Axis.X: "NDOF_Xaxis_2021-03-19.txt",
        #         Axis.Y: "NDOF_Yaxis_2021-03-19.txt",
        #         Axis.Z: "NDOF_Zaxis_2021-03-19.txt"
        #     },
        # },
    }
    # to save different approaches
    saveFig(database)
    # until here to save figures

    #load_reference()

    # set initial config for execution
    experiment = 180
    sensor, axis = Sensor.RELATIVE, Axis.X
    number_experiments = 10
    number_degrees = 18

    base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{experiment}"

    # load the proper file
    file_path = os.path.join(base_path, sensor.value)
    file_path = os.path.join(file_path, axis.value)
    filename = os.path.join(file_path, database[experiment][sensor][axis])
    print(f"filename: {filename}")

    list_file = read_csv(filename)
    #list_distances = compute_distances_ypr_sensor(list_file, axis, number_experiments, number_degrees)
    # the used one
    #list_distances = compute_distances_ypr_rot(list_file, axis, number_experiments, number_degrees)
    #list_distances = compute_distances_ypr_quat(list_file, axis, number_experiments, number_degrees)
    #list_distances = compute_distances_quat(list_file, Axis.X, number_experiments, number_degrees)
    #list_distances = compute_distances_quat_as_theoretical(list_file, axis, number_experiments, number_degrees)
    list_distances = compute_distances_quat_approach1(list_file, axis, number_experiments, number_degrees)
    #list_distances = compute_distances_quat_robot(list_file, Axis.Z, number_experiments, number_degrees)

    #list_distances = compute_distances_quat_theo(list_file)
    text = ""
    if axis == Axis.X:
        text = "_yaw"
    if axis == Axis.Y:
        text = "_pitch"
    if axis == Axis.Z:
        text = "_roll"

    # temporal for drawing purposes
    # list_d = []
    # for entry in list_distances:
    #     if entry["degrees"] < 270:
    #         list_d.append(entry)
    ### used for Axis.x
    #list_d = [entry for entry in list_distances if entry["degrees"] < 270]

    ### used for Axis.y
    # Yaxis-absolute-x-360
    # list_d = list_distances.copy()
    #
    # for index in range(len(list_d)):
    #     if list_d[index]["degrees"] == 90:
    #         list_d[index]["distance_yaw"] = difference_float(180, list_d[index]["distance_yaw"])
    #     elif list_d[index]["degrees"] == 270:
    #         list_d[index]["distance_yaw"] = difference_float(45, list_d[index]["distance_yaw"])
    #     elif 90 < list_d[index]["degrees"] < 270:
    #         list_d[index]["distance_yaw"] = difference_float(180, list_d[index]["distance_yaw"])

    ### used for Axis.y
    # Yaxis-relative-x-180
    # list_d = list_distances.copy()
    #
    # for index in range(len(list_d)):
    #     if list_d[index]["degrees"] == 90:
    #         list_d[index]["distance_roll"] = difference_float(90, list_d[index]["distance_roll"])
    #     elif list_d[index]["degrees"] == 270:
    #         list_d[index]["distance_roll"] = difference_float(45, list_d[index]["distance_roll"])
    #     elif 90 < list_d[index]["degrees"] < 270:
    #         list_d[index]["distance_roll"] = difference_float(180, list_d[index]["distance_roll"])


    #draw_box_plot(list_d, f"distance{text}", title="Error in degrees", y_text=f"error in {axis.name}({text})")
    #draw_box_plot(list_distances, f"distance{text}", title="Error in degrees", y_text=f"error in {axis.name}({text})")
    #
    # list_d = [item for item in list_distances if item["degrees"] > 0]
    # #draw_box_plot(list_distances, f"distance_yaw", title="Error in degrees", y_text=f"error in Yaw")
    # draw_box_plot(list_d, f"distance_yaw", title="Error in degrees", y_text=f"error in Yaw")

    #list_d = [item for item in list_distances if item["degrees"] > 0]
    #draw_box_plot(list_distances, f"distance_pitch", title="Error in degrees", y_text=f"error in Pitch")
    #draw_box_plot(list_d, f"distance_pitch", title="Error in degrees", y_text=f"error in Pitch")

    #draw_box_plot(list_d, f"distance_roll", title="Error in degrees", y_text=f"error in Roll")
    #draw_box_plot(list_distances, f"distance_roll", title="Error in degrees", y_text=f"error in Roll")

    # #output_dir = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{sensor.name.lower()}\\{experiment}\\error_dist_quat\\Axis-X"
    # list_distances = compute_distances_quat(list_file, Axis.X, number_experiments, number_degrees)
    plt.figure()
    draw_box_plot(list_distances, f"distance_ap1", title=f"Error axis {axis.name}", y_text=f"distance in quaternion").savefig("t1.png")
    plt.figure()
    draw_box_plot(list_distances, f"distance", title=f"Error axis {axis.name}",
                  y_text=f"distance in quaternion").savefig("t2.png")
    # #plt.savefig(os.path.join(output_dir, "x.png"), dpi=1200)
    # list_distances = compute_distances_quat(list_file, Axis.Y, number_experiments, number_degrees)
    # draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    # #plt.savefig(os.path.join(output_dir, "y.png"), dpi=1200)
    # list_distances = compute_distances_quat(list_file, Axis.Z, number_experiments, number_degrees)
    # draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    #plt.savefig(os.path.join(output_dir, "z.png"), dpi=1200)
    #draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    # import pandas as pd
    # df_data = pd.read_csv(filename)