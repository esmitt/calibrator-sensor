import sys
import pandas as pd

filename = sys.argv[1]
filename = filename[:-4]
read_file = pd.read_csv (filename + ".csv")
read_file.to_excel (filename + ".xlsx", index = None, header=True)