"""Works in the folder feb 2021"""

import csv
import os

filenames = ['hor_lin_rel-X',
             'hor_lin_rel-Z',
             'hor_lin_rel-Y',
             'hor_lin_abs-X',
             'hor_lin_abs-Z',
             'hor_lin_abs-Y']
filenames = ['hor_lin_abs-X',
             'hor_lin_abs-Z',
             'hor_lin_abs-Y']
# X, MODX, Y, MODY, Z, MODZ
# MODX - angle0x
filenames_output = list()

for filename in filenames:
    output_filename = f"{filename}_post.csv"
    filenames_output.append(output_filename)
    filename = os.path.join("python\\feb 2021",filename + ".csv")
    list_output = []
    print(f"processing {filename}")
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = -1
        n_exp = 0
        baseX, baseY, baseZ = 0, 0, 0
        for row in csv_reader:
            new_row = list()

            if line_count >= 0:
                if line_count % 37 == 0:  # get the degree 0
                    n_exp = int(row[7])
                    baseX = float(row[1])
                    baseY = float(row[2])
                    baseZ = float(row[3])
                    # change to 0 value
                    # if baseX > 300:
                    #     row[1] = f"{360 - baseX}"
                    # if baseZ > 100:
                    #     row[3] = f"{180 - baseZ}"

                degree = int(row[0])
                measureX = float(row[1])
                measureY = float(row[2])
                measureZ = float(row[3])
                measureX_mod = measureX%360
                measureY_mod = measureY%360
                measureZ_mod = measureZ%360
                # if measureZ_mod > 100:
                #     measureZ = 180-measureZ
                #     measureZ_mod = measureZ % 180

                # if I am dealing with the current experiment
                if int(row[7]) == n_exp:
                    diffX = abs((measureX_mod - degree) - baseX%360)
                    row[4] = diffX
                    diffY = abs(baseY%360 - measureY_mod)
                    row[5] = diffY
                    diffZ = abs(baseZ%360 - measureZ_mod)
                    row[6] = diffZ
                    #check Y
                    if diffY > 300:
                        diffY = abs(360 - diffY)
                        row[5] = diffY
                    # check Z
                    if diffZ > 300:
                        diffZ = abs(360 - diffZ)
                        row[6] = diffZ

                row[4] = f"{row[4]}"
                row[5] = f"{row[5]}"
                row[6] = f"{row[6]}"

                new_row.extend([degree, measureX, measureX_mod, measureY, measureY_mod, measureZ, measureZ_mod, row[4], row[5], row[6], int(n_exp)])
                list_output.append(new_row)
            line_count += 1

    with open(output_filename,'w', newline="") as result_file:
        wr = csv.writer(result_file, dialect='excel')
        wr.writerow(["degrees","X","mod_X","Y","mod_Y","Z","mod_Z", "Diff-X","Diff-Y", "Diff-Z","# exp"])
        for row in list_output:
            wr.writerow(row)
