""" Works with mar 2021"""

import seaborn as sns
import os
import pandas as pd
import csv
from matplotlib import pyplot as plt
sns.set() # Setting seaborn as default style even if use only matplotlib

folder = "python\\mar 2021\\agmundet"
header_ndof = ["No. EXP.","Ref (grados)","YAW","PITCH","ROLL","qW","qX","qY","qZ","acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z","mag_x","mag_y","mag_z","sys_cal","accel_cal","gyro_cal","mag_cal"]
header_imu = ["No. EXP.","Ref (grados)","YAW","PITCH","ROLL","qW","qX","qY","qZ","acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z","sys_cal","accel_cal","gyro_cal","mag_cal"]

from enum import Enum
class Axis(Enum):
    x = 0
    y = 1
    z = 2


axis = Axis.x

def errors(filename: str, axis: Axis) -> list:
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        list_output = []

        line_count = 0
        n_exp = 0
        baseX, baseY, baseZ = 0, 0, 0
        for row in csv_reader:
            new_row = list()

            n_exp = int(row[0].strip())
            degree = int(row[1].strip())

            # if line_count % 18 == 0:  # get the degree 0
            if degree == 0:
                baseX = float(row[2])
                baseY = float(row[3])
                baseZ = float(row[4])
                print(row)

            measureX = float(row[2])
            measureY = float(row[3])
            measureZ = float(row[4])
            measureX_mod = measureX % 360
            measureY_mod = measureY % 360
            measureZ_mod = measureZ % 360

            # if I am dealing with the current experiment
            if axis == Axis.x:
                diffX = abs(degree - (measureX_mod - (baseX % 360)) % 360)
                diffY = (measureY_mod - (baseY % 360))
                diffZ = (measureZ_mod - (baseZ % 360))
            if axis == Axis.y:
                diffX = (measureX_mod - (baseX % 360))
                diffY = abs(degree - (measureY_mod - (baseY % 360)) % 360)
                diffZ = (measureZ_mod - (baseZ % 360))
            elif axis == Axis.z:
                diffX = (measureX_mod - (baseX % 360))
                diffY = (measureY_mod - (baseY % 360))
                diffZ = abs(degree - (measureZ_mod - (baseZ % 360)) % 360)
            # print(f"{diffX}: {degree} - ({measureX_mod} - {baseX})")
            # # check Y
            # if diffY > 300:
            #     diffY = abs(360 - diffY)
            # # check Z
            # if diffZ > 300:
            #     diffZ = abs(360 - diffZ)

            new_row.extend(
                [degree, measureX, measureX_mod, measureY, measureY_mod, measureZ, measureZ_mod, diffX,
                 diffY, diffZ, int(n_exp)])
            list_output.append(new_row)
        line_count += 1
    return list_output


def errors_post(filename: str, axis: Axis, trick90_degree: bool) -> list:
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        list_output = []

        fcnFix = lambda x: 360 - x if x > 345 else x%360
        fcnFix90 = lambda degree, x: (90 + x)%360 if degree > 90 else x % 360
        fcnAddBase = lambda base, x: x + base

        fcnRound0 = lambda x : 360 - x if x > 350 else x
        fcnRoundZ = lambda x: 180 - (x%360) if abs(x) > 170 else x

        line_count = 0
        n_exp = 0
        baseX, baseY, baseZ = 0, 0, 0
        for row in csv_reader:
            new_row = list()

            n_exp = int(row[0].strip())
            degree = int(row[1].strip())

            # if line_count % 18 == 0:  # get the degree 0
            if degree == 0:
                # baseX = fcnFix(float(row[2]))
                # baseY = fcnFix90(0, float(row[3]))
                # baseZ = fcnFix(float(row[4]))
                baseX = fcnRound0(float(row[2]))
                baseY = fcnRound0(float(row[3]))
                baseZ = fcnRound0(float(row[4]))
                print(f"{baseX}, {baseY}, {baseZ}")


            measureX = fcnRound0(float(row[2]))
            measureY = fcnRound0(float(row[3]))
            measureZ = fcnRound0(float(row[4]))

            # if I am dealing with the current experiment
            if axis == Axis.x:
                diffX = abs(degree - abs(measureX - baseX))
                diffY = abs(measureY - baseY)
                diffZ = abs(measureZ - baseZ)
            if axis == Axis.y:
                diffX = (measureX - baseX)
                if trick90_degree and degree >= 90:
                    diffY = abs(degree - abs((90 + (90 - measureY)) - baseY))
                else:
                    diffY = abs(degree - abs(measureY - baseY))
                diffZ = abs(fcnRoundZ(measureZ) - baseZ)
            elif axis == Axis.z:
                diffX = abs(measureX - baseX)
                diffY = abs(measureY - baseY)
                diffZ = abs(degree - abs(measureZ - baseZ))
                print(f"degree: {degree}, baseZ:{baseZ}, measureZ_mod: {measureZ}, error-Z:{diffZ}")
            # print(f"{diffX}: {degree} - ({measureX_mod} - {baseX})")
            # # check Y
            # if diffY > 300:
            #     diffY = abs(360 - diffY)
            # # check Z
            # if diffZ > 300:
            #     diffZ = abs(360 - diffZ)

            new_row.extend(
                [degree, measureX, measureX, measureY, measureY, measureZ, measureZ, diffX,
                 diffY, diffZ, int(n_exp)])
            list_output.append(new_row)
        line_count += 1
    return list_output


import math
def errors_tan(filename: str, axis: Axis) -> list:
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        list_output = []

        fcnTan = lambda x : math.tan(math.degrees(x))

        line_count = 0
        n_exp = 0
        baseX, baseY, baseZ = 0, 0, 0
        for row in csv_reader:
            new_row = list()

            n_exp = int(row[0].strip())
            degree = int(row[1].strip())

            # if line_count % 18 == 0:  # get the degree 0
            if degree == 0:
                baseX = float(row[2])
                baseY = float(row[3])
                baseZ = float(row[4])
                print(row)

            measureX = float(row[2])
            measureY = float(row[3])
            measureZ = float(row[4])
            measureX_mod = measureX % 360
            measureY_mod = measureY % 360
            measureZ_mod = measureZ % 360

            # if I am dealing with the current experiment
            if axis == Axis.x:
                diffX = fcnTan(degree) - fcnTan(measureX_mod - (baseX))
                diffY = fcnTan(measureY_mod) - fcnTan(baseY)
                diffZ = fcnTan(measureZ_mod) - fcnTan(baseZ)
            if axis == Axis.y:
                diffX = fcnTan(measureY_mod) - fcnTan(baseY)
                diffY = fcnTan(degree) - fcnTan(measureY_mod - (baseY))
                diffZ = fcnTan(measureZ_mod) - fcnTan(baseZ)
            elif axis == Axis.z:
                diffX = fcnTan(measureY_mod) - fcnTan(baseY)
                diffY = fcnTan(measureY_mod) - fcnTan(baseY)
                diffZ = fcnTan(degree) - fcnTan(measureZ_mod - (baseZ % 360))
            # print(f"{diffX}: {degree} - ({measureX_mod} - {baseX})")
            # # check Y
            # if diffY > 300:
            #     diffY = abs(360 - diffY)
            # # check Z
            # if diffZ > 300:
            #     diffZ = abs(360 - diffZ)

            new_row.extend(
                [degree, measureX, measureX_mod, measureY, measureY_mod, measureZ, measureZ_mod, diffX,
                 diffY, diffZ, int(n_exp)])
            list_output.append(new_row)
        line_count += 1
    return list_output


def write_csv(output_filename: str, list_output: list):
    with open(output_filename,'w', newline="") as result_file:
        wr = csv.writer(result_file, dialect='excel')
        wr.writerow(["degrees","X","mod_X","Y","mod_Y","Z","mod_Z", "Diff-X","Diff-Y", "Diff-Z","# exp"])
        for row in list_output:
            wr.writerow(row)


axis = "Yaxis"
#sensor = "IMU"
sensor = "NDOF"
#base_path = "C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet\\NDOF"
#filename = f"{axis}\\Eje 1 (brazo largo)\\{sensor}_{axis}_2021-03-12.txt"
which_agmundet = 1
if which_agmundet == 1:
    base_path = f"C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet\\{sensor}"
    filename = f"{axis}\\{sensor}_{axis}_2021-03-12.txt"
elif which_agmundet == 2:
    base_path = f"C:\\code\\arduino\\BNO055_AHRS_Python_v2--TxtFrame\\python\\mar 2021\\agmundet 2\\{sensor}"
    filename = f"{axis}\\{sensor}_{axis}_2021-03-19.txt"
filename = os.path.join(base_path, filename)
# read csv
list_errors = errors_post(filename, Axis.y, True)
write_csv(f"{sensor}_{axis}-error.csv", list_errors)

df = pd.read_csv(f"{sensor}_{axis}-error.csv")
#df = pd.read_csv(filename)
sns.color_palette("Set2")
#sns.color_palette("tab10")

name = "absolute" if sensor == "NDOF" else "relative"
evaluating = "X"
ax = sns.boxplot(x="degrees", y="Diff-Z", data=df).set(title=f"{name} in axis {axis[0]}",
                                                                   ylabel=f'Error in {evaluating}')
#ax = sns.boxplot(x="degrees", y=f"Diff-{axis[0]}", data=df).set(title=f"{sensor}_{axis}-error", ylabel='error')
#plt.show()

# # using pandas
# for curr_path, directories, files in os.walk(folder):
#     for file in files:
#         if file.endswith(".txt"):
#             if(file.startswith("IMU")):
#                 full_path = os.path.join(curr_path, file)
#                 dframe = pd.read_csv(full_path, names=header_imu, delimiter=",", index_col=0)
#                 #print(dframe.groupby("No. EXP.").count())
#                 for group in dframe.groupby("No. EXP."):
#                     print(group)
#                 #break
#
#             else:
#                 pass
