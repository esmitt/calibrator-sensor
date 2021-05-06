import math

from errors.reference import compute_reference
from errors.converter import to_quaternion
from sensor.sensor import quat_to_rotmat_sensor, rotation_matrix_to_ypr
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


def compute_distances_ypr_quat(list_sensor: List, axis: Axis, number_experiments: Optional[int] = 10,
                                 number_degrees: Optional[int] = 18) -> List:
    def quaternion_to_ypr(q: Quaternion) -> Tuple:
        rot_mat_sensor0 = quat_to_rotmat_sensor(q.w, q.x, q.y, q.z)
        return rotation_matrix_to_ypr(rot_mat_sensor0)

    list_distances = []

    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        ypr_initial = quaternion_to_ypr(list_experiment[0]['q'])
        list_experiment[0]['distance_yaw'] = 0
        list_experiment[0]['distance_pitch'] = 0
        list_experiment[0]['distance_roll'] = 0
        for degree in range(1, number_degrees):
            ypr = quaternion_to_ypr(list_experiment[degree]['q'])
            distance = difference_tuple(ypr_initial, ypr)
            # distance = np.array(np.zeros(3))
            # distance[0] = max(ypr[0], ypr_initial[0]) - min(ypr[0], ypr_initial[0])
            # distance[1] = max(ypr[1], ypr_initial[1]) - min(ypr[1], ypr_initial[1])
            # distance[2] = max(ypr[2], ypr_initial[2]) - min(ypr[2], ypr_initial[2])
            #distance = np.array(ypr_initial) - np.array(ypr)
            #distance = [d % 360 if d < 0 else d for d in distance]

            # check the swap
            list_experiment[degree]['distance_yaw'] = abs(distance[2])
            list_experiment[degree]['distance_pitch'] = abs(distance[1])
            list_experiment[degree]['distance_roll'] = abs(distance[0])

            if axis == Axis.X:
                #list_experiment[degree]['distance_yaw'] = (list_experiment[degree]["degrees"] - abs(distance[2])) % 360
                list_experiment[degree]['distance_yaw'] = difference_float(list_experiment[degree]["degrees"], abs(distance[2]))
            elif axis == Axis.Y:
                #list_experiment[degree]['distance_pitch'] = (list_experiment[degree]["degrees"] - abs(distance[1])) % 360
                list_experiment[degree]['distance_pitch'] = difference_float(list_experiment[degree]["degrees"], abs(distance[1]))
            elif axis == Axis.Z:
                #list_experiment[degree]['distance_roll'] = (list_experiment[degree]["degrees"] - abs(distance[0])) % 360
                list_experiment[degree]['distance_roll'] = difference_float(list_experiment[degree]["degrees"], abs(distance[0]))
            list_experiment[degree]['distance_yaw'] = abs(list_experiment[degree]['distance_yaw'])
            list_experiment[degree]['distance_pitch'] = abs(list_experiment[degree]['distance_pitch'])
            list_experiment[degree]['distance_roll'] = abs(list_experiment[degree]['distance_roll'])

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

def compute_distances_quat(list_sensor: List, axis :Axis, number_experiments: Optional[int] = 10,
                           number_degrees: Optional[int] = 18) -> List:
    axis_rot = [0]*3
    if axis == Axis.X:
        axis_rot = [1, 0, 0]
    if axis == Axis.Y:
        axis_rot = [0, 1, 0]
    if axis == Axis.Z:
        axis_rot = [0, 0, 1]

    # number_experiments: int = 10
    # number_degrees: int = 18
    list_distances = []
    for index_exp in range(1, number_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor if item['exp'] == index_exp]

        q_initial = list_experiment[0]['q']
        list_experiment[0]['distance'] = 0
        for degree in range(1, number_degrees):
            q_theo = Quaternion(axis=axis_rot + q_initial.axis, degrees=q_initial.degrees + list_experiment[degree]["degrees"])
            q = list_experiment[degree]['q']
            distance = Quaternion.distance(q_theo, q)
            list_experiment[degree]['distance'] = distance

        list_experiment.pop(0)  # remove when degrees == 0
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
def draw_box_plot(list_distances: List, key_to_plot: str, title: str, y_text:str):
    df_list = pd.DataFrame(list_distances, index=None)
    sns.boxplot(x='degrees', y=f'{key_to_plot}', palette="Blues", data=df_list).set(title=title, ylabel=y_text)
    #plt.legend(title='Team', fontsize='2', title_fontsize='12')
    plt.show()


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


if __name__ == "__main__":
    #load_reference()
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
        180: {
            Sensor.RELATIVE: {
                Axis.X: "IMU_Xaxis_2021-03-26.txt",
                Axis.Y: "IMU_Yaxis_2021-03-26.txt",
                Axis.Z: "IMU_Zaxis_2021-03-26.txt"
            },
            Sensor.ABSOLUTE: {
                Axis.X: "NDOF_Xaxis_2021-03-19.txt",
                Axis.Y: "NDOF_Yaxis_2021-03-19.txt",
                Axis.Z: "NDOF_Zaxis_2021-03-19.txt"
            },
        },
    }
    # set initial config for execution
    experiment = 360
    sensor, axis = Sensor.ABSOLUTE, Axis.Z
    number_experiments = 5
    number_degrees = 36

    base_path = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{experiment}"

    # load the proper file
    file_path = os.path.join(base_path, sensor.value)
    file_path = os.path.join(file_path, axis.value)
    filename = os.path.join(file_path, database[experiment][sensor][axis])
    print(f"filename: {filename}")

    list_file = read_csv(filename)
    #list_distances = compute_distances_ypr_sensor(list_file, axis, number_experiments, number_degrees)
    #list_distances = compute_distances_ypr_quat(list_file, axis, number_experiments, number_degrees)
    #list_distances = compute_distances_quat(list_file, Axis.X, number_experiments, number_degrees)
    #list_distances = compute_distances_quat_theo(list_file)
    text = ""
    if axis == Axis.X:
        text = "_yaw"
    if axis == Axis.Y:
        text = "_pitch"
    if axis == Axis.Z:
        text = "_roll"
    # the axis and the others 2
    #draw_box_plot(list_distances, f"distance{text}", title="Error in degrees", y_text=f"error in {axis.name}({text})")
    #draw_box_plot(list_distances, f"distance_yaw", title="Error in degrees", y_text=f"error in {axis.name}({text})")
    #draw_box_plot(list_distances, f"distance_pitch", title="Error in degrees", y_text=f"error in {axis.name}({text})")
    #draw_box_plot(list_distances, f"distance_roll", title="Error in degrees", y_text=f"error in {axis.name}({text})")

    #output_dir = f"D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\python\\errors\\{sensor.name.lower()}\\{experiment}\\error_dist_quat\\Axis-X"
    list_distances = compute_distances_quat(list_file, Axis.X, number_experiments, number_degrees)
    draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    #plt.savefig(os.path.join(output_dir, "x.png"), dpi=1200)
    list_distances = compute_distances_quat(list_file, Axis.Y, number_experiments, number_degrees)
    draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    #plt.savefig(os.path.join(output_dir, "y.png"), dpi=1200)
    list_distances = compute_distances_quat(list_file, Axis.Z, number_experiments, number_degrees)
    draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    #plt.savefig(os.path.join(output_dir, "z.png"), dpi=1200)
    #draw_box_plot(list_distances, f"distance", title="Error q-qtheo", y_text=f"error in quaternion")
    # import pandas as pd
    # df_data = pd.read_csv(filename)