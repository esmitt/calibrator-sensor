from math import sqrt, acos, sin, cos, atan2, radians
from sklearn.metrics import mean_squared_error
import numpy as np
from pyquaternion import Quaternion
import pandas as pd
import typing


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


def get_axis(list_quaternion: list) -> typing.Tuple[np.real, np.array]:
    list_red = remove_outliers(list_quaternion)
    q = average_quaternions(np.asarray(list_red, dtype=np.float32))
    rx, ry, rz = compute_sensor_axis(q[0], q[1], q[2], q[3])
    return q, rz


def measure_error(v1: np.ndarray, v2: np.ndarray) -> float:
    return mean_squared_error(v1, v2)


def quadratic_error(v1: np.ndarray, v2: np.ndarray) -> float:
    return (sqrt(v1[0]*v2[0]) + (v1[1]*v2[1]) + (v1[2]*v2[2]))


# def compute_sensor_axis3(w: float, x: float, y: float, z: float) -> np.ndarray:
#     cos_a = w
#     angle = 2.0 * acos(w)  # returns angle in radians
#     norm = sqrt(x * x + y * y + z * z)
#     if norm == 0:
#         return np.zeros(3)
#     # ux = -x / norm
#     ux = x / norm
#     uy = y / norm
#     uz = z / norm
#     sin_a = sqrt(1.0 - cos_a * cos_a)
#     if abs(sin_a) < 0.0005:
#         sin_a = 1
#     return angle, np.array([ux / sin_a, uy / sin_a, uz / sin_a])

#
# def compute_sensor_axis(w: float, x: float, y: float, z: float) -> np.ndarray:
#     cos_a = w
#     angle = 2.0 * acos(w)  # returns angle in radians
#     norm = sqrt(x * x + y * y + z * z)
#     if norm == 0:
#         return np.zeros(3)
#     # ux = -x / norm
#     ux = x / norm
#     uy = y / norm
#     uz = z / norm
#     sin_a = sqrt(1.0 - cos_a * cos_a)
#     if abs(sin_a) < 0.0005:
#         sin_a = 1
#     return np.array([ux / sin_a, uy / sin_a, uz / sin_a])


def compute_sensor_axis(w: float, x: float, y: float, z: float) -> np.ndarray:
    angle = 2.0 * acos(w)  # returns angle in radians
    norm = sqrt(x * x + y * y + z * z)
    if norm == 0:
        return np.zeros(3)
    # ux = -x / norm
    ux = x / norm
    uy = y / norm
    uz = z / norm
    ry = np.array([ux * uy * (1 - cos(angle)) - uz * sin(angle),
                     cos(angle) + uy * uy * (1 - cos(angle)),
                     uz * uy * (1 - cos(angle)) + ux * sin(angle)])
    rx = np.array([cos(angle) + ux*ux*(1 - cos(angle)),
                   uy*ux*(1 - cos(angle)) + uz*sin(angle),
                   uz*ux*(1 - cos(angle)) - uy*sin(angle)])
    rz = np.array([ux*uz*(1 - cos(angle)) + uy*sin(angle),
                   uy*uz*(1 - cos(angle)) - ux*sin(angle),
                   cos(angle) + uz*uz*(1 - cos(angle))])
    rx[0] = -rx[0]
    ry[0] = -ry[0]
    rz[0] = -rz[0]
    return rx, ry, rz


def compute_sensor_theoretical(theta: float, phi: float, sensor_axis: np.ndarray) -> np.ndarray:
    """
    :param theta is the latitude
    :param phi is the longitude
    :param sensor_axis
    :return numpy array of theoretical sensor for the expected rotations
    """

    def origin_angles(v: np.ndarray) -> float:
        theta_rad = atan2(v[1], v[0])
        theta_deg = (theta_rad / np.pi * 180)
        theta_deg = theta_deg  # + ((0 if theta_rad > 0 else 360) % 360)
        return theta_deg

    theta_zero = origin_angles(sensor_axis)
    theta = theta + theta_zero
    # phi = 90 - phi

    # yaw
    rz = np.array([[np.cos(radians(theta)), -np.sin(radians(theta)), 0],
                   [np.sin(radians(theta)), np.cos(radians(theta)), 0],
                   [0, 0, 1]])
    # pitch;
    ry = np.array([[np.cos(radians(phi)), 0, np.sin(radians(phi))],
                   [0, 1, 0],
                   [-np.sin(radians(phi)), 0, np.cos(radians(phi))]])

    return np.matmul(rz, np.matmul(ry, sensor_axis))


if __name__ == "__main__":
    ll = [[0.1, 0.2, 0.3, 0.4], [0.3, 0.4, 0.5, 0.6], [0.1, 0.2, 0.3, 0.4], [0.3, 0.4, 0.5, 0.6]]
    list_reduced = remove_outliers(ll)
    print(list_reduced)
    npa = np.asarray(list_reduced, dtype=np.float32)
    print(npa)
    t = average_quaternions(npa)
    print(t)
