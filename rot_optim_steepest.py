import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
from matplotlib import pyplot as plt

def calc_quarternion(axis, theta):
    axis = np.array(axis) / np.linalg.norm(axis)
    q1 = np.array([1.,0.,0.,0.])
    q2 = np.hstack([[0.],axis])
    return np.cos(theta / 2) * q1 + np.sin(theta / 2) * q2

def calc_rot_mat_from_omega(omega):
    omega_norm = np.linalg.norm(omega)
    omega_normalized = omega / omega_norm
    q = calc_quarternion(omega_normalized, omega_norm)
    rot = R.from_quat(q).as_matrix()
    return rot

def calc_CD(pts1, pts2, rot = np.eye(3)):
    proj_mat = np.zeros((2,3))
    proj_mat[0][0] = 1.
    proj_mat[1][1] = 1.

    proj_pts1 = np.dot(np.dot(pts1, rot.T), proj_mat.T)
    proj_pts2 = np.dot(np.dot(pts2, rot.T), proj_mat.T)

    tr1 = KDTree(proj_pts1)
    tr2 = KDTree(proj_pts2)

    dist2_from_pts1, idx2_from_pts1 = tr1.query(proj_pts2, k=1)
    dist1_from_pts2, idx1_from_pts2 = tr2.query(proj_pts1, k=1)

    return np.mean(dist1_from_pts2) + np.mean(dist2_from_pts1)

def calc_grad(pts1, pts2, rot):
    proj_mat = np.zeros((2,3))
    proj_mat[0][0] = 1.
    proj_mat[1][1] = 1.

    proj = np.dot(proj_mat.T, proj_mat)

    proj_pts1 = np.dot(np.dot(pts1, rot.T), proj_mat.T)
    proj_pts2 = np.dot(np.dot(pts2, rot.T), proj_mat.T)

    tr1 = KDTree(proj_pts1)
    tr2 = KDTree(proj_pts2)

    _, psi = tr1.query(proj_pts2, k=1)
    _, phi = tr2.query(proj_pts1, k=1)

    N_p = len(pts1)
    N_q = len(pts2)

    g_i = np.zeros(3)
    for i, p in enumerate(pts1):
        g_i += np.cross(np.dot(p - pts2[phi[i]],rot.T),np.dot(np.dot(p - pts2[phi[i]], rot.T), proj.T))

    g_j = np.zeros(3)
    for j, q in enumerate(pts2):
        g_j += np.cross(np.dot(q - pts1[psi[j]],rot.T),np.dot(np.dot(q - pts1[psi[j]], rot.T), proj.T))

    grad = 2. / N_p * g_i + 2. / N_q * g_j
    return grad



def calc_proj_pts(pts_3d):
    proj_mat = np.zeros((2,3))
    proj_mat[0][0] = 1.
    proj_mat[1][1] = 1.
    return np.dot(pts_3d, proj_mat.T)
 

def draw_pts(pts1, pts2, rot = np.eye(3)):
    proj_mat = np.zeros((2,3))
    proj_mat[0][0] = 1.
    proj_mat[1][1] = 1.


    proj_pts1 = np.dot(np.dot(pts1, rot.T), proj_mat.T)
    proj_pts2 = np.dot(np.dot(pts2, rot.T), proj_mat.T)

    fig = plt.figure()
    plt.scatter(proj_pts1[:,0], proj_pts1[:,1])
    plt.scatter(proj_pts2[:,0], proj_pts2[:,1])

    tr1 = KDTree(proj_pts1)
    tr2 = KDTree(proj_pts2)

    _, idx2_from_pts1 = tr1.query(proj_pts2, k=1)
    _, idx1_from_pts2 = tr2.query(proj_pts1, k=1)
    for i in range(len(idx2_from_pts1)):
        j = idx2_from_pts1[i]
        plt.plot([proj_pts1[j,0], proj_pts2[i,0]],
                    [proj_pts1[j,1], proj_pts2[i,1]], "-b")
    
    for i in range(len(idx1_from_pts2)):
        j = idx1_from_pts2[i]
        plt.plot([proj_pts2[j,0], proj_pts1[i,0]],
                    [proj_pts2[j,1], proj_pts1[i,1]], "-r")

def create_pts(angle):
    N_pts = 400
    t = np.linspace(0,2 * np.pi, N_pts)
    noise_rnd = np.random.randn(t.shape[0])
    r = 5.0 + noise_rnd * 0.1
    x = r * np.cos(t) + 12
    y = r * np.sin(t) + 14
    z = np.ones(x.shape)
    pts1 = np.vstack([x,y,z]).T

    noise_rnd = np.random.randn(t.shape[0])
    r = 5.0 + noise_rnd * 0.1
    x = r * np.cos(t) + 12
    y = r * np.sin(t) + 14
    z = np.ones(x.shape)
    pts2 = np.vstack([x,y,50.0*z]).T

    axis_init = np.array([2., 5., 2])
    theta_init = np.radians(angle)
    q_init = calc_quarternion(axis_init, theta_init)

    rot_init = R.from_quat(q_init)
    trans_mat = np.eye(4)
    trans_mat[:3,:3] = rot_init.as_matrix()
    print(trans_mat)

    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(pts1)
    pcd1.paint_uniform_color([1.,0.,0.])
    pcd1 = copy.deepcopy(pcd1).transform(trans_mat)

    pcd2.points = o3d.utility.Vector3dVector(pts2)
    pcd2.paint_uniform_color([0.,1.,0.])
    pcd2 = copy.deepcopy(pcd2).transform(trans_mat)

    pts1 = np.array(pcd1.points)
    pts2 = np.array(pcd2.points)
    return pts1, pts2

def parse_vec2mat(vec):
    mat = np.array([[     0., -vec[2],  vec[1]],
                    [ vec[2],      0., -vec[0]],
                    [-vec[1],  vec[0],      0.]])
    return mat

def calc_hessian(pts1, pts2, rot):
    proj_mat = np.zeros((2,3))
    proj_mat[0][0] = 1.
    proj_mat[1][1] = 1.

    proj = np.dot(proj_mat.T, proj_mat)

    proj_pts1 = np.dot(np.dot(pts1, rot.T), proj_mat.T)
    proj_pts2 = np.dot(np.dot(pts2, rot.T), proj_mat.T)

    tr1 = KDTree(proj_pts1)
    tr2 = KDTree(proj_pts2)

    _, psi = tr1.query(proj_pts2, k=1)
    _, phi = tr2.query(proj_pts1, k=1)

    N_p = len(pts1)
    N_q = len(pts2)


    mat_i = np.zeros((3,3))
    for i, p in enumerate(pts1):
        q = pts2[phi[i]]
        vec1 = np.dot(p-q,np.dot(rot.T, proj.T))
        mat1 = parse_vec2mat(vec1)
        vec2 = np.dot(p-q,rot.T)
        mat2 = parse_vec2mat(vec2)

        mat_i += mat1 @ mat2 - mat2 @ proj @ mat2
    
    mat_j = np.zeros((3,3))
    for j, q in enumerate(pts2):
        p = pts1[psi[j]]
        vec1 = np.dot(q-p,np.dot(rot.T, proj.T))
        mat1 = parse_vec2mat(vec1)
        vec2 = np.dot(q-p,rot.T)
        mat2 = parse_vec2mat(vec2)

        mat_j += mat1 @ mat2 - mat2 @ proj @ mat2

    mat = 1. / N_p * mat_i + 1. / N_q * mat_j
    #mat = mat_i + mat_j
    
    return mat
        



def main():
    
    pts1, pts2 = create_pts(10)
    #frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    #o3d.visualization.draw_geometries([frame_mesh, pcd1, pcd2])

    rot_init = np.eye(3)

    axis_init = np.array([2., 1., 2])
    theta_init = np.radians(0.)
    q_init = calc_quarternion(axis_init, theta_init)

    rot = rot_init#R.from_quat(q_init).as_matrix()


    print(calc_CD(pts1, pts2,rot))
    print(calc_grad(pts1,pts2,rot))
    print(calc_hessian(pts1, pts2, rot))

    draw_pts(pts1,pts2,rot)

    eps = 1e-4
    for i in range(10000):
        J = calc_CD(pts1,pts2,rot)
        g = calc_grad(pts1, pts2, rot)
        print(f"iter={i}")
        print(f"J={J}")
        print(f"g={g}")
        print(f"rot{rot}")

        #steepest algorithm (using linear search)
        alpha = 100.
        l = 1e-6
        gamma = (np.sqrt(5.) - 1.) / 2.

        #a: lower_bound, b: upper bound
        a = 0.
        b = alpha

        alpha1 = a + (1. - gamma) * (b - a)
        alpha2 = a + gamma * (b - a)

        rot_omega1 = calc_rot_mat_from_omega(alpha1 * g)
        rot_omega2 = calc_rot_mat_from_omega(alpha2 * g)
        rot_tilde1 = rot_omega1 @ rot
        rot_tilde2 = rot_omega2 @ rot
        J_new1 = calc_CD(pts1, pts2, rot_tilde1)
        J_new2 = calc_CD(pts1, pts2, rot_tilde2)

        for j in range(100):
            #print(f"Current [a,b]=[{a},{b}],\t J={(J_new1,J_new2)}")
            if b - a < l:
                alpha = (a + b) / 2
                break
            else:
                rot_omega1 = calc_rot_mat_from_omega(alpha1 * g)
                rot_omega2 = calc_rot_mat_from_omega(alpha2 * g)
                rot_tilde1 = rot_omega1 @ rot
                rot_tilde2 = rot_omega2 @ rot
                J_new1 = calc_CD(pts1, pts2, rot_tilde1)
                J_new2 = calc_CD(pts1, pts2, rot_tilde2)

                if J_new1 >= J_new2:
                    a = alpha1
                    b = b
                    alpha1 = alpha2
                    alpha2 = a + gamma * (b - a)
                else:
                    a = a
                    b = alpha2
                    alpha2 = alpha1
                    alpha1 = a + (1. - gamma) * (b - a)

        #calc phi1
        omega = alpha * g
        omega_norm = np.linalg.norm(omega)
        omega = omega / omega_norm
        q = calc_quarternion(omega, omega_norm)
        rot_omega = R.from_quat(q).as_matrix()
        rot_tilde = rot_omega @ rot
        J_new = calc_CD(pts1,pts2,rot_tilde)
        rot = rot_tilde
        if i%10 == 0:
            pass
            draw_pts(pts1, pts2, rot)
        if J < eps:
            break



    
    print("EOF")






if __name__ == "__main__": 
    main()