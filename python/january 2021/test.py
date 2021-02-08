import os
from sensor import *
import json

#folders = ["python\\january 2021\\position 1", "python\\january 2021\\position 2", "python\\january 2021\\new_test"]
folders = ["python\\january 2021\\yaw",
           "python\\january 2021\\pitch",
           "python\\january 2021\\roll"]

def save_dic_to_csv(dic_errors: dict, filename: str):
    # saving into a file
    with open(filename, "w") as file1:
        file1.write("yaw\n")
        file1.write("angle,mean,std,min,max\n")
        for index_angle in angles:
            if index_angle == 0:
                continue

            str_out = f'{index_angle-10} to {index_angle},{dic_errors[index_angle]["yaw"]["mean"]}, {dic_errors[index_angle]["yaw"]["std"]},' \
                      f'{dic_errors[index_angle]["yaw"]["min"]},{dic_errors[index_angle]["yaw"]["max"]}\n'
            file1.write(str_out)

        file1.write("pitch\n")
        for index_angle in angles:
            if index_angle == 0:
                continue
            str_out = f'{index_angle-10} to {index_angle},{dic_errors[index_angle]["pitch"]["mean"]}, {dic_errors[index_angle]["pitch"]["std"]},' \
                      f'{dic_errors[index_angle]["pitch"]["min"]},{dic_errors[index_angle]["pitch"]["max"]}\n'
            file1.write(str_out)

        file1.write("roll\n")
        for index_angle in angles:
            if index_angle == 0:
                continue
            str_out = f'{index_angle-10} to {index_angle},{dic_errors[index_angle]["roll"]["mean"]}, {dic_errors[index_angle]["roll"]["std"]},' \
                      f'{dic_errors[index_angle]["roll"]["min"]},{dic_errors[index_angle]["roll"]["max"]}\n'
            file1.write(str_out)

def get_entries(folder: str) -> list:
    folder = os.path.join(os.getcwd(), folder)
    list_entries= list()

    with os.scandir(folder) as entries:
        for entry in entries:
                list_entries.append(entry)
    return list_entries

def get_values_experiments(list_experiments: list):
    list_mat_world = list()
    list_mat_sensor = list()
    list_ypr_world = list()
    # each experiment contains all angles
    for experiment in list_experiments:
        rotation_mat = [quat_to_rotmat_world(q) for q in experiment]
        list_ypr_world.append([rotation_matrix_to_ypr(np.array(mat)) for mat in rotation_mat])
        list_mat_world.append(rotation_mat)
        list_mat_sensor.append([quat_to_rotmat_sensor(q[0], q[1], q[2], q[3]) for q in experiment])

    # convert rotation matrices to yaw, pitch, roll
    # list_ypr_world = list()
    # for exp in list_mat_world:
    #     #list_ypr_world.append([rotation_matrix_to_ypr(np.array(mat)) for mat in list_mat_world])
    #     list_ypr = list()
    #     for mat in exp:
    #         list_ypr.append(rotation_matrix_to_ypr(np.array(mat)))
    #     list_ypr_world.append(list_ypr)

    return list_mat_world, list_ypr_world


str_evaluate = ["yaw", "pitch", "roll"]
index_folder = 2

list_experiments = [load_quaternions(filename.path) for filename in get_entries(folders[index_folder])]
assert(len(list_experiments) > 0), "list_experiments variable is empty"
number_experiments = len(list_experiments)

angles = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]
n_angles = len(angles)
dic_errors = dict()

list_mat_world, list_ypr_world = get_values_experiments(list_experiments)

list_errors_exp = []
if str_evaluate[index_folder] == "yaw":
    # for yaw
    for index_exp in range(number_experiments):
        list_errors = []
        for index in range(1, n_angles):
            current = list_ypr_world[index_exp][index]
            previous = list_ypr_world[index_exp][index - 1]
            yaw_error = (10.0 - np.abs(current[0] - previous[0]))
            if np.abs(yaw_error) > 90:
                yaw_error = (yaw_error)%360
            pitch_error = np.abs(current[1] - previous[1])%180
            roll_error = np.abs(current[2] - previous[2])
            if roll_error > 90:
                roll_error = 360 - roll_error
            list_errors.append([np.abs(yaw_error), np.abs(pitch_error), np.abs(roll_error)])
        list_errors_exp.append(list_errors)
elif str_evaluate[index_folder] == "pitch":
    for index_exp in range(number_experiments):
        list_errors = []
        for index in range(1, n_angles):
            current = list_ypr_world[index_exp][index]
            previous = list_ypr_world[index_exp][index - 1]
            yaw_error = (10.0 - np.abs(current[0] - previous[0]))
            if np.abs(yaw_error) > 90:
                yaw_error = yaw_error%90
            pitch_error = np.abs(current[1] - previous[1])
            roll_error = np.abs(current[2] - previous[2])
            #list_errors.append([np.abs(yaw_error), np.abs(pitch_error), np.abs(roll_error)])
            list_errors.append([np.abs(pitch_error), np.abs(yaw_error), np.abs(roll_error)])
        list_errors_exp.append(list_errors)
    # for pitch
    # list_yaw = []
    # list_pitch = []
    # list_roll = []
    # for index_exp in range(number_experiments):
    #     n_angles = len(list_ypr_world[index_exp])
    #     for index in range(1, n_angles):
    #         current = list_ypr_world[index_exp][index]
    #         previous = list_ypr_world[index_exp][index - 1]
    #         yaw_error = np.abs((10.0 - np.abs(current[0] - previous[0])))
    #         if yaw_error > 90:
    #             yaw_error = 360 - yaw_error
    #         pitch_error = np.abs(current[1] - previous[1])
    #         roll_error = ((np.abs(current[2] - previous[2])))
    #         print(yaw_error, pitch_error, roll_error)
elif str_evaluate[index_folder] == "roll":
    for index_exp in range(number_experiments):
        list_errors = []
        for index in range(1, n_angles):
            current = list_ypr_world[index_exp][index]
            previous = list_ypr_world[index_exp][index - 1]
            #yaw_error = np.abs(current[0] - previous[0])
            yaw_error = 90 - (10.0 - np.abs(current[0] - previous[0])) % 90
            pitch_error = np.abs(current[1] - previous[1])
            roll_error = 90 - (10.0 - np.abs(current[2] - previous[2]))%90
            list_errors.append([np.abs(yaw_error), np.abs(pitch_error), np.abs(roll_error)])
        list_errors_exp.append(list_errors)
    # for roll
    # list_yaw = []
    # list_pitch = []
    # list_roll = []
    # for index_exp in range(number_experiments):
    #     n_angles = len(list_ypr_world[index_exp])
    #     for index in range(1, n_angles):
    #         current = list_ypr_world[index_exp][index]
    #         previous = list_ypr_world[index_exp][index - 1]
    #         yaw_error = np.abs(current[0] - previous[0])
    #         pitch_error = np.abs(current[1] - previous[1])
    #         roll_error = np.abs(current[2] - previous[2])
    #         print(yaw_error, pitch_error, roll_error)

# create a dictionary grouped by yaw, pitch and roll
# outside the position 0
for index in range(n_angles - 1):
    yaw_error = []
    pitch_error = []
    roll_error = []
    for index_exp in range(number_experiments):
        yaw_error.append(list_errors_exp[index_exp][index][0])
        pitch_error.append(list_errors_exp[index_exp][index][1])
        roll_error.append(list_errors_exp[index_exp][index][2])
    dic_errors[angles[index + 1]] = {
        "yaw":
            {
                "mean": np.mean(yaw_error),
                "std": np.std(yaw_error),
                "min": np.min(yaw_error),
                "max": np.max(yaw_error)
            }
        , "pitch":
            {
                "mean": np.mean(pitch_error),
                "std": np.std(pitch_error),
                "min": np.min(pitch_error),
                "max": np.max(pitch_error)
            }
        , "roll":
            {
                "mean": np.mean(roll_error),
                "std": np.std(roll_error),
                "min": np.min(roll_error),
                "max": np.max(roll_error)
            }
    }

save_dic_to_csv(dic_errors, f"python\\{str_evaluate[index_folder]}.csv")

M = [[1, 2, 3], [1, 1, 1], [2, 2, 2]]
for i in range(len(M)):
    sfil = sum(M[i])
col = 2
scol, sfil = 0, sum(M[2])
for i in range(len(M)):
    scol += M[i][col]