from pyquaternion import Quaternion
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib

sns.set_theme(style="darkgrid")

qx = Quaternion(axis=[1, 0, 0], degrees=20)
qy = Quaternion(axis=[0, 1, 0], degrees=20)
qz = Quaternion(axis=[0, 0, 1], degrees=20)

start_degree = 20
q_initial = qz

list_distances = list()
for degrees in range(-10, 10):
    q = Quaternion(axis=q_initial.axis, degrees=q_initial.degrees + degrees)
    list_distances.append({"degree": degrees,
                           "distance": Quaternion.distance(q_initial, q)
                           })

df = pd.DataFrame(list_distances, index=None)
sns.lineplot(data=df, x="degree", y="distance")

plt.show()
