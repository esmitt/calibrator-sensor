from src.sensor.sensor import Sensor, Axis, read_csv, rotation_matrix_to_ypr
from typing import List, Optional
from pyquaternion import Quaternion
from math import degrees
from numpy.linalg import norm, inv
from numpy import matmul

import os
import matplotlib
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from src.errors.reference import compute_reference

sns.set() # Setting seaborn as default style even if use only matplotlib
#sns.set_theme(style="whitegrid")
sns.despine()
matplotlib.use('TkAgg' )

from src.errors.reference import compute_reference

sns.set() # Setting seaborn as default style even if use only matplotlib
#sns.set_theme(style="whitegrid")
sns.despine()
#matplotlib.use('TkAgg' )


def distance_between_quaternions(q0: Quaternion, q1: Quaternion,dist_type='euclidean'):

    if dist_type=='euclidean':
        return min(degrees(Quaternion.absolute_distance(q0, q1)), degrees(Quaternion.absolute_distance(q0, -q1)))
    else:
        return min(degrees(Quaternion.distance(q0, q1)), degrees(Quaternion.distance(q0, -q1)))


def difference_two_signs(degrees: int, value1: float, value2: float) -> float:
    """Check if two values have different sign to make the correct subtraction"""
    if (value1 * value2) < 0:
        if value1 < 0:
            value1 = degrees + (degrees - abs(value1))
        if value2 < 0:
            value2 = degrees + (degrees - abs(value2))
    return abs(max(value1, value2) - min(value1, value2))


def difference_two_values(v1: float, v2: float) -> float:
    return (max(v1, v2) - min(v1, v2))%360


def compute_distance(list_sensor_data: List, axis: Axis, n_experiments: int, n_degrees: int,dist_type='euclidean') -> List:
    # chose axis to spin
    axis_quaternion = [0]
    if axis == Axis.X:
        axis_quaternion = [1, 0, 0]
    if axis == Axis.Y:
        axis_quaternion = [0, 1, 0]
    if axis == Axis.Z:
        axis_quaternion = [0, 0, 1]

    list_distances = list()

    for index_exp in range(1, n_experiments + 1):
        # extract experiment according to the index
        list_experiment = [item for item in list_sensor_data if item['exp'] == index_exp]  # <-- contains a experiment
        # capture from sensor, the initial position
        q_initial = list_experiment[0]['q']
        ### used in paper
        ypr_initial = list_experiment[0]['ypr']
        ### used in paper

        list_experiment[0]['distance'] = 0

        # iterate over all degrees
        for index in range(1, n_degrees):
            degree = list_experiment[index]["degrees"]
            # current quaternion capture from sensor
            q = list_experiment[index]['q']

            ### used in paper
            # difference between ypr
            yaw = float(list_experiment[index]['ypr'][1])
            if degree < 90:
                yaw_theo = float(ypr_initial[1]) + degree
                list_experiment[index]['distance_ypr'] = difference_two_values(yaw, yaw_theo)
            elif degree < 270:
                yaw_theo = float(ypr_initial[1]) - degree
                list_experiment[index]['distance_ypr'] = 180 - difference_two_values(yaw, yaw_theo)
            elif 270 <= degree < 360:
                yaw_theo = float(ypr_initial[1]) + degree
                list_experiment[index]['distance_ypr'] = 360 - difference_two_values(yaw, yaw_theo)

            # use the difference in yaw
            roll, pitch, yaw = list_experiment[index]['ypr']
            #yaw_theo = ypr_initial[1] + degree
            yaw_theo = ypr_initial[1]
            #distance = difference_two_values(pitch, yaw_theo)
            list_experiment[index]['distance_yaw'] = pitch
            list_experiment[index]['distance_yaw_quat'] = rotation_matrix_to_ypr(q.rotation_matrix)[1]

            # HACK: used in paper (submitted) #####
            # compute difference in ypr using conversion from quaternion to ypr
            ypr_initial_from_q = rotation_matrix_to_ypr(q_initial.rotation_matrix)
            ypr_from_q = rotation_matrix_to_ypr(q.rotation_matrix)

            yaw_initial, pitch_initial, roll_initial = ypr_initial_from_q[2], ypr_initial_from_q[1], ypr_initial_from_q[0]
            yaw, pitch, roll = ypr_from_q[2], ypr_from_q[1], ypr_from_q[0]

            if axis == Axis.X:
                if degree < 90:
                    list_experiment[index]['dyaw'] = difference_two_values(yaw_initial - list_experiment[index]['degrees'], yaw)
                elif 90 <= degree <= 180:
                    list_experiment[index]['dyaw'] = difference_two_values(yaw_initial - list_experiment[index]['degrees'], yaw)
                elif 180 < degree < 270:
                    list_experiment[index]['dyaw'] = 360 - difference_two_values(yaw_initial - list_experiment[index]['degrees'], yaw)
                else:
                    list_experiment[index]['dyaw'] = difference_two_values(yaw_initial - list_experiment[index]['degrees'], yaw)

                if list_experiment[index]['dyaw'] > 300:
                    list_experiment[index]['dyaw'] = 360 - list_experiment[index]['dyaw']

                list_experiment[index]['dpitch'] = difference_two_values(pitch_initial, pitch)

                if list_experiment[index]['dpitch'] > 300:
                    list_experiment[index]['dpitch'] = 360 - list_experiment[index]['dpitch']


                list_experiment[index]['droll'] = difference_two_values(roll_initial, roll)

            if axis == Axis.Y:
                list_experiment[index]['dyaw'] = difference_two_values(yaw_initial, yaw)

                if degree < 90:
                    list_experiment[index]['dpitch'] = difference_two_values(pitch_initial - degree, pitch)
                elif 90 <= degree < 270:
                    list_experiment[index]['dyaw'] = 180 - difference_two_values(yaw_initial, yaw)
                    list_experiment[index]['dpitch'] = 180 - difference_two_values(pitch_initial + degree, pitch)
                else:
                    list_experiment[index]['dpitch'] = difference_two_values(pitch_initial - degree, pitch)

                if 90 <= degree < 270:
                    list_experiment[index]['droll'] = 180 - difference_two_values(roll_initial, roll)
                elif degree == 270:
                    list_experiment[index]['droll'] = 360 - difference_two_values(roll_initial, roll)
                else:
                    list_experiment[index]['droll'] = difference_two_values(roll_initial, roll)

            if axis == Axis.Z:
                list_experiment[index]['dyaw'] = difference_two_values(yaw_initial, yaw)
                list_experiment[index]['dpitch'] = difference_two_values(pitch_initial, pitch)
                list_experiment[index]['droll'] = difference_two_values(roll_initial - degree, roll)

            ### used in paper

            # compute theoretical quaternion
            q_theo = Quaternion(axis=axis_quaternion, degrees=-degree)

            # distance 1
            q_temp = q_initial.inverse * q
            list_experiment[index]['distance1_paper'] = distance_between_quaternions(q_theo, q_temp,dist_type)
            # distance 2
            # q_temp = Quaternion(matrix=matmul(inv(q_initial.rotation_matrix), q.rotation_matrix))
            # list_experiment[index]['distance2_paper'] = distance_between_quaternions(q_theo, q_temp)
            # distance 3 (not considered for decoupling)
            # r_temp = q_theo.rotation_matrix - matmul(inv(q_initial.rotation_matrix), q.rotation_matrix)
            # list_experiment[index]['distance3_newpaper'] = norm(r_temp)

            # decoupling in angle + axis
            # angle_diff = difference_two_signs(degree, q_temp.degrees, degrees(q_theo.angle))
            angle_diff = abs( q_temp[0]- q_theo[0])
            axis_diff = norm(q_temp.axis - q_theo.axis)
            list_experiment[index]['angle1_paper'] = angle_diff
            list_experiment[index]['axis1_paper'] = axis_diff
            #list_experiment[index]['angle2_paper'] = angle_diff
            #list_experiment[index]['axis2_paper'] = axis_diff

        list_distances.extend(list_experiment)
    return list_distances


def draw_box_plot(list_entry: List, key_to_plot: str, title: str):
    """Contains the key key_to_plot in axis Y vs. `degrees` in axis X"""
    df_list = pd.DataFrame(list_entry, index=None)
    with sns.axes_style("whitegrid"):
        #sns.set_context("paper", rc={"font.size": 30, "axes.titlesize": 16, "axes.labelsize": 12})
        sns.set_context("paper", font_scale=3.5, rc={"lines.linewidth": 2.5})
        #sns.set_context("paper")
        #sret = sns.boxplot(x='degrees', y=f'{key_to_plot}', palette="Blues", data=df_list, showfliers=False)
        sret = sns.boxplot(x='degrees', y=f'{key_to_plot}', data=df_list, showfliers=False)  #, palette="light:black",palette="dark:red"
        #sret = sns.pointplot(x='degrees', y=f'{key_to_plot}', data=df_list,showfliers=False)
        #sret = sns.lineplot(x='degrees', y=f'{key_to_plot}', data=df_list)
        #sret.set(title=title, ylabel=y_text)
        #sret.set(title=title)
        #sret.tick_params(labelsize=18)
        sret.set_xticks(range(2, 35, 3))
        #sret.set_xticks(range(30, 360, 30))
        sret.set_xticklabels(range(30, 360, 30))
        sret.set(xlabel=None, ylabel=None)
        #sret.set_ylim(-100, 100)
        #, showfliers=False
        #plt.legend(title='Team', fontsize='2', title_fontsize='12')
        #plt.show()
        plt.tight_layout()
        return sret.get_figure()


def check_create_dir(path_dir: str):
    """If directory doesnt exists, then this create it"""
    if not os.path.isdir(path_dir):
        os.mkdir(path_dir)


def saving_figure(list_dist: List, png_filename: str, output_path: str, key_metric: str, title: Optional[str] = ""):
    """Outliers are out in figure"""
    plt.figure()
    output_dir = os.path.join(output_path, key_metric)
    check_create_dir(output_dir)
    output_file = os.path.join(output_dir, png_filename)
    draw_box_plot(list_distances, key_metric, title).savefig(output_file)
    filter_dist = [l for l in list_dist if l["degrees"] > 0 and l["degrees"] < 360 ]
    #draw_box_plot(filter_dist, key_metric, title, "degrees").savefig(output_file)
    ### used in paper
    #draw_box_plot(list_distances, key_metric, "Error in degrees for axis Y", "Error in degrees")
    plt.show()
    plt.waitforbuttonpress()
    #### used in paper


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

        #for sensor in [Sensor.RELATIVE, Sensor.ABSOLUTE]:
        for sensor in [Sensor.ABSOLUTE]:
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
                #plt.figure()
                saving_figure(list_distances, filename, output_path, "distance1_paper", f"Quaternion distance in {axis.value} ({sensor.name.lower()})")
                #saving_figure(list_distances, filename, output_path, "droll", "")
                # saving_figure(list_distances, filename, output_path, "dpitch",
                #               f"Quaternion distance in {axis.value} ({sensor.name.lower()})")
                # saving_figure(list_distances, filename, output_path, "droll",
                #               f"Quaternion distance in {axis.value} ({sensor.name.lower()})")
                saving_figure(list_distances, filename, output_path, "angle1_paper", f"Angle distance in {axis.value} ({sensor.name.lower()})")
                saving_figure(list_distances, filename, output_path, "axis1_paper", f"Axis distance in {axis.value} ({sensor.name.lower()})")

                ##### for paper
                #df_list = pd.DataFrame(list_distances, index=None)
                #with sns.axes_style("whitegrid"):
                #     sns.set_context("paper", font_scale=3.5, rc={"lines.linewidth": 3.0})
                #     sret = sns.pointplot(join=False, x='degrees', y='distance_yaw', data=df_list, color="blue")
                #     sret = sns.pointplot(join=False,x='degrees', y='distance_yaw_quat', data=df_list,  color="red")

                #     sret.set_xticks(range(2, 35, 3))
                     #sret.set_xticks(range(30, 360, 30))
                 #    sret.set_xticklabels(range(30, 360, 30))
                 #    sret.set(xlabel=None, ylabel=None)
                 #    plt.tight_layout()
                #plt.show()
                ##### for paper

                #saving_figure(list_distances, filename, output_path, "distance_yaw_quat",
                #              f"Axis distance in {axis.value} ({sensor.name.lower()})")


                #saving_figure(filename, output_path, "distance2_paper")
                #saving_figure(filename, output_path, "angle2_paper")
                #saving_figure(filename, output_path, "axis2_paper")
                ### used in paper
                #saving_figure(filename, output_path, "distance_ypr")
                ### used in paper
                plt.close("all")
