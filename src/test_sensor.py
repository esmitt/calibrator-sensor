"""
Exp1: Yaw validation
     Compare R_[0 0 1](yaw_i) * Rq0 with Rq_i
Theoretical Motion-Yaw: rot axis is g=[0,0,1]
by experimental design sphere is on ground
Exp2: Pitch validation
     Compare R_[Rq0_y](picth_i) * Rq0 with Rq_i
Theoretical Motion-Pitch: rot axis is y-axis of intial quaternion (noted Rq0_y).
We assume that it is aligned with sphere
Exp3: Roll validation (sensor is moved along its y axis picth_i degrees)
     Compare R_[Rq0_x](picth_i) * Rq0 with Rq_i
Theoretical Motion-Roll: rot axis is x-axis of intial quaternion (noted Rq0_x).
We assume that it is aligned with sphere
"""

from sensor import *
import sys
import os
import pickle

def load_quaternions(filename: str = "quaternions.q") -> list:
    open_file = open(filename, "rb")
    loaded_list = pickle.load(open_file)
    open_file.close()
    return loaded_list

# prints quaternions in list
def print_list(qlist: list):
    for index in range(len(qlist)):
        print(f"{index}: {qlist[index]}")

def print_yaws(initial_q, ylist: list):
    print(f"initial quaternion:\n{initial_q}")
    for index in range(len(ylist)):
        print(f"{index+1}: {ylist[index]}")

def get_theoretical_rotmat(angle: float, vth, rotmat0):
    theoretical_rot = get_rotmat(radians(angle), vth[0],vth[1],vth[2])
    return (np.matmul(theoretical_rot,rotmat0))

def rot_axis_angle_to_ypr(degrees: float, v: list, source: np.ndarray) -> tuple:
    """
    Function to rotate `source` an amount of `degrees` around the vector `v `
    :param degrees: a value in degrees
    :param v: axis vector where to rotate
    :param source: vector to be rotated
    :return: a tuple value representing yaw, pitch, roll
    """
    rotation_matrix = get_rotmat(radians(degrees), v[0], v[1], v[2])
    return rotation_matrix_to_ypr(np.matmul(rotation_matrix, source))

def error_ypr(rotmat: np.ndarray, ypr_theoretical: tuple) -> tuple:
    """
    Function to return the absolute error between two yaw, pitch, roll values
    :param rotmat: rotation matrix where a yaw, pitch, roll is computed.
    :param ypr_theoretical: theoretical yaw, pitch, roll to compare
    :return:
    """
    ypr = rotation_matrix_to_ypr(rotmat)
    error = np.abs(np.subtract(ypr, ypr_theoretical))
    return error

def error_in_rotsensor_ypr(rotmat: np.ndarray, rotmat_theoretical: np.ndarray) -> tuple:
    """
    Function to return error between theoretical and sensor rotations:
        rotmat_theoretical*rotmat0=rotmat
    :param rotmat: rotation given by sensor.
    :param rotmat0: sensor initial rotation matrix
    :param rotmat_theoretical: theoretical rotation matrix
    :return:
    """
    ypr = rotation_matrix_to_ypr(rotmat)
    ypr_theoretical = rotation_matrix_to_ypr(rotmat_theoretical)  # mew added
    error = np.abs(np.subtract(ypr , ypr_theoretical))
    return error

def debora_test():
    filenames = ['quaternions-2020-12-17--11-22-16.q',
                 'quaternions-2020-12-17--11-27-33.q']

    # Pitch Validation
    index = 0
    filename = os.path.join(result_dir, filenames[index])
    quaternions = load_quaternions(filename=filename)
    rot_mat_world = [quat_to_rotmat_world(q) for q in quaternions]
    rot_mat_sensor = [quat_to_rotmat_sensor(q[0],q[1],q[2],q[3]) for q in quaternions]


    sensor_yaws = [rotation_matrix_to_ypr(rot) for rot in rot_mat_world]
    sensor_yaws_sensor = [rotation_matrix_to_ypr(rot) for rot in rot_mat_sensor]

    angles = [0, 20, 40, 60, 80]
    angles = [0, -20, -40, -60, -80]
    angles_pitch = [0, -20, -40, -60, -80]
    angles_pitch_yaw = [0, -10, -20, -30, -40]

    # Tenemos un problema de orientacion: Necesitamos saber dnd monto el agujerito

    # Theoretical Motion-Roll: rot axis is x-axis of intial quaternion
    rotmat0 = rot_mat[0]
    theoretical_rot_roll = [get_theoretical_rotmat(angle, rotmat0[:, 0], rotmat0) for angle in angles]
    theoretical_yaws_roll = [rotation_matrix_to_ypr(theoretical_rot) for theoretical_rot in theoretical_rot_roll]

    # Exp3b: Theoretical Motion-Roll-Yaw:
    rotmat0 = rot_mat[1]
    theoretical_rot_roll_yaw = [get_theoretical_rotmat(angle, [0, 0, 1], rotmat0) for angle in angles_pitch_yaw]
    theoretical_yaws_roll_yaw = [rotation_matrix_to_ypr(theoretical_rot) for theoretical_rot in
                                 theoretical_rot_roll_yaw]

    # Exp2: Theoretical Motion-Pitch: rot axis is y-axis of intial quaternion.
    # We assume that it is aligned with sphere
    rotmat0 = rot_mat[0]
    theoretical_rot_pitch = [get_theoretical_rotmat(angle, rotmat0[:, 1], rotmat0) for angle in angles]
    theoretical_yaws_pitch = [rotation_matrix_to_ypr(theoretical_rot) for theoretical_rot in theoretical_rot_pitch]

    # Exp2b: Theoretical Motion-Pitch-Yaw:
    rotmat0 = rot_mat[1]
    theoretical_rot_pitch_yaw = [get_theoretical_rotmat(angle, [0, 0, 1], rotmat0) for angle in angles_pitch_yaw]
    theoretical_yaws_pitch_yaw = [rotation_matrix_to_ypr(theoretical_rot) for theoretical_rot in
                                  theoretical_rot_pitch_yaw]

    # Exp1: Theoretical Motion-Yaw: rot axis is g=[0,0,1] by experimental design (sphere is on ground)
    #
    g = [0, 0, 1]
    rotmat0 = rot_mat[0]
    theoretical_rot_yaw = [get_theoretical_rotmat(angle, g, rotmat0) for angle in angles]
    theoretical_yaws_yaw = [rotation_matrix_to_ypr(theoretical_rot) for theoretical_rot in theoretical_rot_yaw]

    # print(theoretical_yaws)
    # print(sensor_yaws)

    # print(f"yaw:{theoretical_yaws[0]}, picth:{theoretical_yaws[1]}, roll:{theoretical_yaws[2]}")
    print(f"yaw:{sensor_yaws[0]}, picth:{sensor_yaws[1]}, roll:{sensor_yaws[2]}")

    index = 0  # Error in Pitch
    err = abs(np.array(theoretical_yaws_pitch) - np.array(sensor_yaws))
    err = np.array(err)
    print(f" mean:{np.mean(err[:, 1])} ,std:{np.std(err[:, 1])}")

def read_list_quaternions(result_path: str) -> list:
    x = [f.name for f in os.scandir(result_path) if f.is_file()]
    return x

if __name__ == '__main__':
    #code_main_dir = r'J:\Experiments\Endoscopy\Navigation\Sensor\Code\python'
    code_main_dir = os.getcwd()

    sys.path.append(code_main_dir)

    #result_dir = r'J:\Experiments\Endoscopy\Navigation\Sensor\Results\test_Dec_2020'
    #result_dir = os.path.join(code_main_dir, 'test_Dec_2020')
    result_dir = os.path.join(code_main_dir, 'experiments/exp1/1.1')

    filenames = ['quaternions-2020-12-17--11-22-16.q',
                 'quaternions-2020-12-17--11-27-33.q']
    #filenames = read_list_quaternions(result_dir)
    filenames = 'quaternions-2020-12-21--18-14-09.q'
    filenames = 'quaternions-2020-12-21--18-42-29.q'
    filenames = "python/quaternions-2021-01-22--10-44-17.q"

    q = load_quaternions(os.path.join(code_main_dir,filenames))
    # list_q = 0list()
    # for file in filenames:
    #     list_q.append(load_quaternions(os.path.join(result_dir, file)))
    # print_list(list_q)