from sensor import *
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
    error = np.abs(np.subtract(ypr , ypr_theoretical))
    return error

if __name__ == '__main__':
    filenames = ['test_Dec_2020/quaternions-2020-12-17--11-22-16.q',
                 'test_Dec_2020/quaternions-2020-12-17--11-27-33.q']
    index = 1
    quaternions = load_quaternions(filename=filenames[index])
    rot_mat = [quat_to_rotmat_world(q) for q in quaternions]
    angles = [0, 20, 40, 60 , 80]

    i = 1
    theoretical_rot = rot_axis_angle_to_ypr(angles[i], [1, 0, 0], rot_mat[i])
    print(error_ypr(rot_mat[i], theoretical_rot))

    i = 2
    theoretical_rot = rot_axis_angle_to_ypr(angles[i], rot_mat[0][:, 1], rot_mat[i])
    print(error_ypr(rot_mat[i], theoretical_rot))
