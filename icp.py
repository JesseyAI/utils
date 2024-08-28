import numpy as np
import open3d as o3d

def load_point_cloud(file_path):
    """ Load point cloud from a text file """
    point_cloud = np.loadtxt(file_path)
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_cloud))

def preprocess_point_cloud(pcd, voxel_size):
    """ Downsample the point cloud and estimate normals """
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_size * 2, max_nn=30))
    return pcd_down

def compute_icp(source, target, voxel_size):
    """ Compute the ICP transformation from source to target """
    threshold = voxel_size * 10
    trans_init = np.eye(4)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    return reg_p2p.transformation

def apply_transformation(point_cloud, transformation):
    """ Apply transformation to the point cloud """
    points = np.asarray(point_cloud.points)
    ones = np.ones((points.shape[0], 1))
    points_hom = np.hstack([points, ones])
    transformed_points = (transformation @ points_hom.T).T
    point_cloud.points = o3d.utility.Vector3dVector(transformed_points[:, :3])
    return point_cloud

def save_point_cloud(point_cloud, file_path):
    """ Save point cloud to a text file """
    np.savetxt(file_path, np.asarray(point_cloud.points), fmt='%.6f')

def main():
    np.set_printoptions(suppress=True)  # Disable scientific notation for print

    his_file = "/home/qijie/Data/Nebula200/nebula220/20240805/opt/1.txt"
    loop_file = "/home/qijie/Data/Nebula200/nebula220/20240805/opt/2.txt"
    refine_file = "/home/qijie/Data/Nebula200/nebula220/20240805/opt/refine.txt"

    his_pcd = load_point_cloud(his_file)
    loop_pcd = load_point_cloud(loop_file)

    voxel_size = 0.05  # Set the voxel size for downsampling
    his_pcd_down = preprocess_point_cloud(his_pcd, voxel_size)
    loop_pcd_down = preprocess_point_cloud(loop_pcd, voxel_size)

    transformation = compute_icp(loop_pcd_down, his_pcd_down, voxel_size)

    R = transformation[:3, :3]
    t = transformation[:3, 3]

    print("Transformation matrix:")
    print(transformation)
    print("\nRotation matrix (R):")
    print(R)
    print("\nTranslation vector (t):")
    print(t)

    refined_pcd = apply_transformation(loop_pcd, transformation)
    save_point_cloud(refined_pcd, refine_file)

if __name__ == "__main__":
    main()
