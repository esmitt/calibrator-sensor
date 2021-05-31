""" reference.csv is in the data folder"""
from pyquaternion import Quaternion
import pandas as pd
from tqdm import tqdm
from src.errors.converter import to_quaternion


def compute_reference(yaw0: float, pitch0: float, roll0: float) -> pd.DataFrame:
    q_initial = to_quaternion(yaw0, pitch0, roll0)
    degree_u, degree_t = 0, +360


    list_distances = list()
    for yaw in tqdm(range(degree_u, degree_t)):
        for pitch in range(degree_u, degree_t):
            for roll in range(degree_u, degree_t):
                q = to_quaternion(yaw + yaw0, pitch + pitch0, roll + roll0)
                distance = Quaternion.distance(q, q_initial)
                list_distances.append({"yaw": yaw, "pitch": pitch, "roll": roll, "distance": distance})
    df = pd.DataFrame(list_distances, index=None)
    return df