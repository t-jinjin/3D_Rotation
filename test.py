from pathlib import Path
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import ipywidgets as widgets

def load_point_cloud(path: Path) -> np.ndarray:
    assert path.is_absolute() and path.exists()
    ply = o3d.io.read_point_cloud(str(path))
    return np.asarray(ply.points)


a = load_point_cloud(Path("./bunny/reconstruction/bun_zipper_res2.ply").resolve())


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(a[:,0],a[:,1],a[:,2], marker = 'o', color="b")



print("EoF")
