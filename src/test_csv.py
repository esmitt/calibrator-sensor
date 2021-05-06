import csv
import os

filenames = ['hor_lin_rel-X-2021-02-09--23-39-17', 'hor_lin_rel-Y-2021-02-10--09-26-55',
             'hor_lin_rel-Z-2021-02-10--13-30-36', 'hor_lin_abs-X-2021-02-10--17-52-33',
             'hor_lin_abs-Y-2021-02-10--22-05-05', 'hor_lin_abs-Z-2021-02-10--16-14-18']

for filename in filenames:
    output_filename = f"{filename[:13]}.csv"
    filename = os.path.join("python",filename + ".csv")
    list_output = []

    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = -1
        n_exp = 0
        baseX, baseY, baseZ = 0, 0, 0
        for row in csv_reader:
            if line_count >= 0:
                if line_count %37 == 0:  # get the degree 0
                    n_exp = int(row[7])
                    baseX = float(row[1])
                    baseY = float(row[2])
                    baseZ = float(row[3])
                    # change to 0 value
                    if baseX > 300:
                        row[1] = f"{360 - baseX}"

                degree = int(row[0])
                measureX = float(row[1])
                measureY = float(row[2])
                measureZ = float(row[3])

                if int(row[7]) == n_exp:
                    diffX = abs((measureX - degree) - baseX)
                    if abs(diffX) > 300:
                        diffX = abs(360 - diffX)
                    row[4] = diffX

                    diffY = abs(measureY - baseY)
                    if abs(diffY) > 100:
                        diffY = abs(180 - diffY)
                    if abs(diffY) > 100:
                        diffY = 180 -abs(diffY)
                    row[5] = diffY

                    diffZ = abs(measureZ - baseZ)
                    if abs(diffZ) > 100:
                        diffZ = abs(180 - diffZ)
                    if abs(diffZ) > 100:
                        diffZ = 180 -abs(diffZ)
                    row[6] = diffZ

                row[4] = f"{row[4]}"
                row[5] = f"{row[5]}"
                row[6] = f"{row[6]}"
                list_output.append(row)
            line_count += 1

    with open(output_filename,'w', newline="") as result_file:
        wr = csv.writer(result_file, dialect='excel')
        wr.writerow(["degrees","X","Y","Z","Diff-X","Diff-Y", "Diff-Z","# exp"])
        for row in list_output:
            wr.writerow(row)
