import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt

def calc_quarternion(axis, theta):
    axis = np.array(axis) / np.linalg.norm(axis)
    q1 = np.array([1.,0.,0.,0.])
    q2 = np.hstack([[0.],axis])
    return np.cos(theta / 2) * q1 + np.sin(theta / 2) * q2


def main():
    N_pts = 1000 
    t = np.linspace(0,2 * np.pi, N_pts)
    r = 5.0
    x = r * np.cos(t)
    y = r * np.sin(t)
    z = np.ones(x.shape)
    pts1 = np.vstack([x,y,z]).T

    pts2 = np.vstack([x,y,5.0*z]).T

    axis = [2., 1., 2]
    theta = np.radians(10.)
    q = calc_quarternion(axis, theta)

    rot = R.from_quat(q)
    rot_mat = rot.as_matrix()
    trans_mat = np.eye(4)
    trans_mat[:3,:3] = rot_mat
    print(trans_mat)

    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pts1)
    pcd1.paint_uniform_color([1.,0.,0.])
    pcd1 = copy.deepcopy(pcd1).transform(trans_mat)

    pcd2.points = o3d.utility.Vector3dVector(pts2)
    pcd2.paint_uniform_color([0.,1.,0.])
    pcd2 = copy.deepcopy(pcd2).transform(trans_mat)

    frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    o3d.visualization.draw_geometries([frame_mesh, pcd1, pcd2])


    pts1 = np.array(pcd1.points)
    pts2 = np.array(pcd2.points)

    






if __name__ == "__main__":
    main()