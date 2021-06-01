from math import sqrt, acos, sin, cos, atan2, radians, degrees
from sklearn.metrics import mean_squared_error
import numpy as np
import math
from pyquaternion import Quaternion
import pandas as pd
import pickle
from enum import Enum
import csv
from typing import List, Dict


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


# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def average_quaternions(quat: np.array) -> np.real:
    # Number of quaternions to average
    M = quat.shape[0]
    A = np.zeros(shape=(4, 4))

    for i in range(0, M):
        q = quat[i, :]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    # return np.real(eigenVectors[:, 0].A1)
    return np.real(np.asarray(eigenVectors[:, 0]).ravel())


def remove_outliers(list_values: list, min_quantile: float = 0.1, max_quantile: float = 0.9) -> list:
    list_angles = []

    # convert each quaternion into its angle
    for quat in list_values:
        list_angles.append(Quaternion(quat).angle)

    # convert into pandas Series to filter according quantile
    list_series = pd.Series(list_angles)
    list_outliers = list_series.between(list_series.quantile(min_quantile),
                                        list_series.quantile(max_quantile))

    return_list = []
    for index in range(len(list_outliers)):
        if list_outliers[index]:
            return_list.append(list_values[index])

    return return_list


def measure_error(v1: np.ndarray, v2: np.ndarray) -> float:
    return mean_squared_error(v1, v2)


def quadratic_error(v1: np.ndarray, v2: np.ndarray) -> float:
    return (sqrt(v1[0]*v2[0]) + (v1[1]*v2[1]) + (v1[2]*v2[2]))


def get_rotmat(angle: float, ux: float, uy: float, uz: float) -> np.array:
     ry = np.array([ux * uy * (1 - cos(angle)) - uz * sin(angle),
                      cos(angle) + uy * uy * (1 - cos(angle)),
                      uz * uy * (1 - cos(angle)) + ux * sin(angle)])
     rx = np.array([cos(angle) + ux*ux*(1 - cos(angle)),
                    uy*ux*(1 - cos(angle)) + uz*sin(angle),
                    uz*ux*(1 - cos(angle)) - uy*sin(angle)])
     rz = np.array([ux*uz*(1 - cos(angle)) + uy*sin(angle),
                    uy*uz*(1 - cos(angle)) - ux*sin(angle),
                    cos(angle) + uz*uz*(1 - cos(angle))])

     return np.array([rx, ry, rz])


#https://github.com/adafruit/Adafruit_BNO055/blob/master/utility/quaternion.h
# v[0] is applied 1st about z (ie, roll)
# v[1] is applied 2nd about y (ie, pitch)
# v[2] is applied 3rd about x (ie, yaw)
def quat_to_rotmat_sensor_adafruit(w: float, x: float, y: float, z: float) -> np.ndarray:
    sqw = w * w
    sqx = x * x
    sqy = y * y
    sqz = z * z
    ret = np.array([0] * 3)

    ret[0] = math.atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw))
    ret[1] = math.asin(-2.0 * (x * z - y * w) / (sqx + sqy + sqz + sqw))
    ret[2] = math.atan2(2.0 * (y * z + x * w), (-sqx - sqy + sqz + sqw))

    return ret


def quat_to_rotmat_sensor(w: float, x: float, y: float, z: float) -> np.ndarray:
    angle = 2.0 * acos(w)  # returns angle in radians
    norm = sqrt(x * x + y * y + z * z)
    if norm == 0:
        return np.array([np.zeros(3), np.zeros(3), np.zeros(3)])
    # ux = -x / norm
    ux = x / norm
    uy = y / norm
    uz = z / norm
    ax = np.array([cos(angle) + ux * ux * (1 - cos(angle)),
                   uy * ux * (1 - cos(angle)) + uz * sin(angle),
                   uz * ux * (1 - cos(angle)) - uy * sin(angle)])

    ay = np.array([ux * uy * (1 - cos(angle)) - uz * sin(angle),
                     cos(angle) + uy * uy * (1 - cos(angle)),
                     uz * uy * (1 - cos(angle)) + ux * sin(angle)])

    az = np.array([ux*uz*(1 - cos(angle)) + uy*sin(angle),
                   uy*uz*(1 - cos(angle)) - ux*sin(angle),
                   cos(angle) + uz*uz*(1 - cos(angle))])

    return np.transpose(np.array([ax, ay, az]))


# return the 3 columns
def compute_axis(w: float, x: float, y: float, z: float) -> tuple:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    R = quat_to_rotmat_sensor(w, -x, z, y)  # columns
    #RT = np.matmul(np.linalg.inv(T), np.matmul(R, T))
    #RT = np.matmul(np.linalg.inv(T), R)
    RT = np.matmul(T, R)
    return RT[:, 0], RT[:, 1], RT[:, 2]


def quaternion2rotmatrix_world1(w: float, x: float, y: float, z: float) -> np.ndarray:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    R = quat_to_rotmat_sensor(w, -x, z, y)  # columns
    #RT = np.matmul(np.linalg.inv(T), np.matmul(R, T))
    #RT = np.matmul(np.linalg.inv(T), R)
    RT = np.matmul(T, R) # original
    #RT = np.matmul(R, T)
    #RT= np.matmul(np.linalg.inv(T), np.matmul(R, T))
    return RT


def quaternion2rotmatrix_world2(w: float, x: float, y: float, z: float) -> np.ndarray:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    R = quat_to_rotmat_sensor(w, -x, z, y)  # columns
    #RT = np.matmul(np.linalg.inv(T), np.matmul(R, T))
    #RT = np.matmul(np.linalg.inv(T), R)
    # RT = np.matmul(T, R) # original
    RT = np.matmul(R, T)
    #RT= np.matmul(np.linalg.inv(T), np.matmul(R, T))
    return RT


def quaternion2rotmatrix_world3(w: float, x: float, y: float, z: float) -> np.ndarray:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    R = quat_to_rotmat_sensor(w, -x, z, y)  # columns
    #RT = np.matmul(np.linalg.inv(T), np.matmul(R, T))
    #RT = np.matmul(np.linalg.inv(T), R)
    # RT = np.matmul(T, R) # original
    #RT = np.matmul(R, T)
    RT = np.matmul(np.linalg.inv(T), np.matmul(R, T))
    return RT


def quat_to_rotmat_world(q: list) -> np.ndarray:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    R = quat_to_rotmat_sensor(q[0], -q[1], q[3], q[2])  # columns
    RT = np.matmul(T, R)  # originaq
    return RT


def sensor2world(v: np.array) -> np.array:
    T = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    return np.matmul(T, v)


def compute_axis_theoretical(theta: float, phi: float, ax, ay, az) -> np.ndarray:
    theta_rad = atan2(ax[1], ax[0])
    theta_deg = (theta_rad / np.pi * 180)
    theta = theta + theta_deg
    # yaw
    global Rz, Ry
    Rz = np.array([[np.cos(radians(theta)), -np.sin(radians(theta)), 0],
                   [np.sin(radians(theta)), np.cos(radians(theta)), 0],
                   [0, 0, 1]])
    # pitch;
    Ry = np.array([[np.cos(radians(phi)), 0, np.sin(radians(phi))],
                   [0, 1, 0],
                   [-np.sin(radians(phi)), 0, np.cos(radians(phi))]])
    Rt = np.matmul(Rz, Ry)
    Tax = np.matmul(Rt, ax)
    Tay = np.matmul(Rt, ay)
    Taz = np.matmul(Rt, az)

    return Tax, Tay, Taz


def rotation_matrix_to_ypr(Rt)-> tuple:
    yaw = (degrees(atan2(Rt[1, 0], Rt[0, 0]))+360)%360
    roll = (degrees(atan2(Rt[2, 1], Rt[2, 2]))+360)%360
    pitch = (degrees(atan2(-Rt[2, 0], sqrt(pow(Rt[0, 0], 2) + pow(Rt[1, 0], 2))))+360)%360
    return yaw, pitch, roll


def calibrate_angle0_world(q0: tuple, qlist: list) -> list:
    #Rq0 = quat_to_rotmat_world(q0[0], q0[1], q0[2], q0[3])
    Rq0 = quat_to_rotmat_world(q0)
    list_yaw = list()

    for q in qlist:
        #Rq = quat_to_rotmat_world(q[0], q[1], q[2], q[3])
        Rq = quat_to_rotmat_world(q)
        #Rt = np.matmul(Rq, np.linalg.inv(Rq0))
        Rt = np.matmul(np.linalg.inv(Rq0), Rq)
        yaw = degrees(atan2(Rt[1, 0], Rt[0, 0]))
        roll = degrees(atan2(Rt[2, 1], Rt[2, 2]))
        pitch = degrees(atan2(-Rt[2, 0], sqrt(pow(Rt[0, 0], 2) + pow(Rt[1, 0], 2))))
        list_yaw.append({"yaw":f"{yaw:.3f}", "pitch":f"{pitch:.3f}", "roll":f"{roll:.3f}"})
    return list_yaw


def fun(list_q: list, quaternion2rotmatrix) -> list:
    list_angles = list()
    for q in list_q:
        Rt = quaternion2rotmatrix(q[0], q[1], q[2], q[3])
        yaw = degrees(atan2(Rt[1, 0], Rt[0, 0]))
        pitch = degrees(atan2(-Rt[2, 0], sqrt(pow(Rt[0, 0], 2) + pow(Rt[1, 0], 2))))
        roll = degrees(atan2(Rt[2, 1], Rt[2, 2]))
        list_angles.append({"yaw":f"{yaw:.3f}", "pitch":f"{pitch:.3f}", "roll":f"{roll:.3f}"})
    return list_angles


def load_quaternions(filename: str = "quaternions.q") -> list:
    open_file = open(filename, "rb")
    loaded_list = pickle.load(open_file)
    open_file.close()
    return loaded_list


if __name__ == "__main__":
    ll = [[0.1, 0.2, 0.3, 0.4], [0.3, 0.4, 0.5, 0.6], [0.1, 0.2, 0.3, 0.4], [0.3, 0.4, 0.5, 0.6]]
    list_reduced = remove_outliers(ll)
    print(list_reduced)
    npa = np.asarray(list_reduced, dtype=np.float32)
    print(npa)
    t = average_quaternions(npa)
    print(t)
