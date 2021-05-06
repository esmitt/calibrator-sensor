from src.sensor.sensor import Sensor, Axis, read_csv
from typing import List
from pyquaternion import Quaternion
from math import degrees
from numpy.linalg import norm, inv
from numpy import matmul

import os
import matplotlib
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

sns.set() # Setting seaborn as default style even if use only matplotlib
sns.set_theme(style="darkgrid")
matplotlib.use('TkAgg' )


def distance_between_quaternions(q0: Quaternion, q1: Quaternion):
    return min(degrees(Quaternion.distance(q0, q1)), degrees(Quaternion.distance(q0, -q1)))


def draw_box_plot(list_entry: List, key_to_plot: str, title: str, y_text:str):
    """Contains the key key_to_plot in axis Y vs. `degrees` in axis X"""
    df_list = pd.DataFrame(list_entry, index=None)
    sns.set_context("paper", rc={"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 5})
    sret = sns.boxplot(x='degrees', y=f'{key_to_plot}', palette="Blues", data=df_list, showfliers=False)
    sret.set(title=title, ylabel=y_text)
    #, showfliers=False
    #plt.legend(title='Team', fontsize='2', title_fontsize='12')
    #plt.show()
    return sret.get_figure()


def difference_two_signs(degrees: int, value1: float, value2: float) -> float:
    if (value1 * value2) < 0:
        if value1 < 0:
            value1 = degrees + (degrees - abs(value1))
        if value2 < 0:
            value2 = degrees + (degrees - abs(value2))
    return abs(max(value1, value2) - min(value1, value2))


def compute_distance(list_sensor_data: List, axis: Axis, n_experiments: int, n_degrees: int) -> List:
    # chose axis to spin
    axis_quaternion = [0]
    if axis == Axis.X:
        axis_quaternion = [1, 0, 0]
    if axis == Axis.Y:
        axis_quaternion = [0, 1, 0]
    if axis == Axis.Z:
        axis_quaternion = [0, 0, 1]

    list_distances = []

    for index_exp in range(1, n_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor_data if item['exp'] == index_exp]  # <-- contains a experiment
        # capture from sensor, the initial position
        q_initial = list_experiment[0]['q']
        list_experiment[0]['distance'] = 0

        # iterate over all degrees
        for index in range(1, n_degrees):
            degree = list_experiment[index]["degrees"]
            # current quaternion capture from sensor
            q = list_experiment[index]['q']

            # compute theoretical quaternion
            q_theo = Quaternion(axis=axis_quaternion, degrees=-degree)
            # distance 1
            q_temp = q_initial.inverse * q
            list_experiment[index]['distance1_paper'] = distance_between_quaternions(q_theo, q_temp)
            # distance 2
            # key_distance = "distance2"
            # q_temp = Quaternion(matrix=matmul(inv(q_initial.rotation_matrix), q.rotation_matrix))
            # list_experiment[index]['distance2_paper'] = distance_between_quaternions(q_theo, q_temp)
            # distance 3 (not considered for decoupling)
            # r_temp = q_theo.rotation_matrix - matmul(inv(q_initial.rotation_matrix), q.rotation_matrix)
            # list_experiment[index]['distance3_newpaper'] = norm(r_temp)

            # decoupling in angle + axis
            angle = difference_two_signs(degree, q_temp.degrees, degrees(q_theo.angle))
            list_experiment[index]['angle1_paper'] = angle
            list_experiment[index]['axis1_paper'] = norm(q_temp.axis - q_theo.axis)
        list_distances.extend(list_experiment)
    return list_distances

"""
Es desacoplando el q_temp:
2. Axis error as distance(q_temp.angle,axis_theo)
"""

def check_create_dir(path_dir: str):
    if not os.path.isdir(path_dir):
        os.mkdir(path_dir)


def saving_figure(filename: str, output_path: str, key_metric: str):
    plt.figure()
    output_dir = os.path.join(output_path, key_metric)
    check_create_dir(output_dir)
    output_file = os.path.join(output_dir, filename)
    draw_box_plot(list_distances, key_metric, key_metric, "Quat distance").savefig(output_file)


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
        }
    }
    experiments_path = "D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\data\\errors"
    output_path = "D:\\code\\BNO055_AHRS_Python_v2--TxtFrame\\data\\errors\\paper"
    check_create_dir(output_path)

    # iterate over the database
    #experiments = list(database.keys())
    experiments = [360]
    for experiment in experiments:
        if experiment == 180:
            number_experiments = 10
            number_degrees = 19
        else:    # experiment == 360:
            number_experiments = 5
            number_degrees = 36

        for sensor in [Sensor.RELATIVE, Sensor.ABSOLUTE]:
            for axis in [Axis.X, Axis.Y, Axis.Z]:

                # load the proper file
                file_path = os.path.join(experiments_path, str(experiment))
                file_path = os.path.join(file_path, sensor.value)
                file_path = os.path.join(file_path, axis.value)
                filename = os.path.join(file_path, database[experiment][sensor][axis])
                print(f"reading: {filename}")
                list_content = read_csv(filename)

                # compute distance stored into a List of dict
                list_distances = compute_distance(list_content, axis, number_experiments, number_degrees)

                # draw output
                filename = f"{axis.value}-{sensor.name.lower()}-{experiment}.png"

                saving_figure(filename, output_path, "distance3_paper")
                saving_figure(filename, output_path, "angle3_paper")
                saving_figure(filename, output_path, "axis3_paper")
                plt.close("all")