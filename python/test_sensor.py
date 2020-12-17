from sensor import *
import pickle

def load_quaternions(filename="quaternions.q"):
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
    rotation_matrix = get_rotmat(radians(degrees), v[0], v[1], v[2])
    return rotation_matrix_to_ypr(np.matmul(rotation_matrix, source))

def error_ypr(rotmat: np.ndarray, ypr_theoretical: tuple) -> tuple:
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

    get_rotmat(radians(degrees), v[0], v[1], v[2])

    i = 1
    theoretical_rot = rot_axis_angle_to_ypr(angles[i], [1, 0, 0], rot_mat[i])
    print(error_ypr(rot_mat[i], theoretical_rot))

    i = 2
    theoretical_rot = rot_axis_angle_to_ypr(angles[i], rot_mat[0][:, 1], rot_mat[i])
    print(error_ypr(rot_mat[i], theoretical_rot))

    # for index in range(1, len(angles)):
    #     rot_theoretical = rot_axis_angle_to_ypr(-angles[index], [0, 0, 1], rot_mat[index - 1])
    #     print(rot_theoretical)
    #     #print(rot_axis_angle_to_ypr(rot_mat))
    #     #print(error_ypr(rot_mat[index], rot_theoretical))
    #     print(rot_axis_angle_to_ypr(-40 - 20, rot_mat[0][:, 1], rot_mat[0]))

def test():
    filenames = ['quaternions-2020-12-16--13-27-22.q',
                 'quaternions-2020-12-16--13-33-17.q',
                 'quaternions-2020-12-16--13-43-10.q',
                 'quaternions-2020-12-16--17-31-17.q']

    index = 3
    list_quaternion = load_quaternions(filename=filenames[index])
    #print_list(list_quaternion)
    #print()
    initial_q = list_quaternion[0]
    list_q = list_quaternion
    yaws = calibrate_angle0_world(initial_q, list_q)
    #print_yaws(initial_q, yaws)

    yaws = calibrate_angle0_world(initial_q, list_q)
    yaws_0 = calibrate_angle0_world((1, 0, 0, 1), list_q)
    for index in range(len(yaws)):
        y = yaws[index]
        y0 = yaws_0[index]
        print(f"{y} --> {y0}")

    list_quaternion = load_quaternions('quaternions-2020-12-16--19-32-02.q')
    #list_quaternion = load_quaternions('quaternions-2020-12-16--20-07-16.q')
    q0, q1, q2 = list_quaternion[0], list_quaternion[1], list_quaternion[2]
    Rq0, Rq1, Rq2 = quat_to_rotmat_world(q0), quat_to_rotmat_world(q1), quat_to_rotmat_world(q2)

    # giro en z de q1
    yawsRq2Thz = rot_axis_angle_to_ypr(-20, [0, 0, 1], Rq1)
    print(error_ypr(Rq2, yawsRq2Thz))

    # giro en y
    yawsRq1Thy = rot_axis_angle_to_ypr(180 - 40, Rq0[:, 1], Rq0)
    print(error_ypr(Rq1, yawsRq1Thy)) #1

    yawsRq2Thy = rot_axis_angle_to_ypr(-40 - 20, Rq0[:, 1], Rq0)
    print(error_ypr(Rq2, yawsRq2Thy)) #1


###########
# # yawsRq0=rotation_matrix_to_ypr(Rq0)
# # yawsRq1=rotation_matrix_to_ypr(Rq1)
# # yawsRq2=rotation_matrix_to_ypr(Rq2)
#
# # giro en z de q1
# yawsRq2Thz = rot_theoretical_ypr(-20, [0, 0, 1], Rq1)
#
# #print(yawsRq2Thz, y1)
#
# print(error_ypr(Rq2, yawsRq2Thz))
#
# # print(abs(yawsRq2[0]-yawsRq2Thz[0])) # error en yaw
# # print(abs(yawsRq2[1]-yawsRq2Thz[1])) # error en picth
#
# # giro en y
# yawsRq1Thy = rot_theoretical_ypr(180 - 40, Rq0[:, 1], Rq0)
# print(error_ypr(Rq1, yawsRq1Thy)) #1
# #print(abs(yawsRq1[1]- yawsRq1Thy[1]))
# yawsRq2Thy = rot_theoretical_ypr(-40 - 20, Rq0[:, 1], Rq0)
# print(error_ypr(Rq2, yawsRq2Thy)) #1
# #print(abs(yawsRq2[1]- yawsRq2Thy[1]))