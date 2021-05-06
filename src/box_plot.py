"""Works in the folder feb 2021"""

#box plot
import seaborn as sns
import os
import pandas as pd
from matplotlib import pyplot as plt
sns.set() # Setting seaborn as default style even if use only matplotlib

sns.set_theme(style="whitegrid")

filenames_output = ["hor_lin_rel-X_post.csv",
                    "hor_lin_rel-Y_post.csv",
                    "hor_lin_rel-Z_post.csv",
                    "hor_lin_abs-X_post.csv",
                    "hor_lin_abs-Y_post.csv",
                    "hor_lin_abs-Z_post.csv"]

filename = os.path.join("python\\feb 2021", filenames_output[2])
#filename = os.path.join(filenames_output[3])
df = pd.read_csv(os.path.join("python\\feb 2021", "hor_cur_rel-X-2021-03-01--20-07-14.csv"))
#df = pd.read_csv(filename)
sns.color_palette("Set2")
sns.boxplot(x="degrees", y="Diff-X", data=df)

#
df["Observer"] = df["# exp"].apply(lambda x: "esmitt" if x<=10 else "manuel")
data_df_mix = df[(df["degrees"]%10 == 0) & (df["degrees"] < 360)]
sns.color_palette("Set2")
ax = sns.boxplot(x="degrees", y="Diff-Z",
            hue="Observer",
            data=data_df_mix)
title = filename[-22:-9]
ax.set_title(title)
ax.set_ylabel(f"error in Z")
sns.despine(offset=10, trim=True)

# until here

for filename in filenames_output:
    #filename = os.path.join("python\\feb 2021", filename + ".csv")
    df = pd.read_csv(filename)

#fig, axes = plt.subplots(2, 1, sharey=True)
#fig.suptitle('X-axis relative')

    data_df_one = df[(df["degrees"]%20 == 0) & (df["degrees"] < 360) & (df["# exp"] <= 10)]

# filter_degree = df["degrees"].apply(lambda x: True if x%20 == 0 and x < 360 else False)
# data_df_one = df[filter_degree]["# exp"].apply(lambda x: True if x<=10 else False)
# df.loc[data_df_one]

#sns.boxplot(ax=axes[0], x='degrees', y='Diff-X', data=data_df)
#axes[0].set_title("esmitt")

    data_df_two = df[(df["degrees"]%20 == 0) & (df["degrees"] < 360) & (df["# exp"] > 10)]
#sns.boxplot(ax=axes[1], x='degrees', y='Diff-X', data=data_df)
#axes[1].set_title("manuel")

cdf = pd.concat([data_df_one, data_df_two])
mdf = pd.melt(cdf, id_vars=["degrees"], var_name=["Diff-X"])

data_df_one = df[(df["degrees"]%20 == 0) & (df["degrees"] < 360)]
data_df_one["Observer"] = data_df_one[(data_df_one["# exp"] > 10)]
sns.boxplot( data=mdf)

# new approach, de 20 en 20
df["Observer"] = df["# exp"].apply(lambda x: "esmitt" if x<=10 else "manuel")
data_df_mix = df[(df["degrees"]%20 == 0) & (df["degrees"] < 360)]
sns.color_palette("Set2")
sns.boxplot(x="degrees", y="Diff-X",
            hue="Observer",
            data=data_df_mix)
sns.despine(offset=10, trim=True)



plt.show()

plt.clf()
plt.close()

def split_list(list_values: list) -> (list, list):
    pass