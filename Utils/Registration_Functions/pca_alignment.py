import numpy as np
import open3d as o3d


def align_point_cloud(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """
    Aligns a point cloud using principal component analysis (PCA).

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.

    Returns:
        o3d.geometry.PointCloud: The aligned point cloud.
    """
    # Extract points from the point cloud
    points = np.asarray(pcd.points)

    # Compute the mean of the points
    mean = np.mean(points, axis=0)

    # Calculate centered points
    centered_points = points - mean

    # Compute the covariance matrix
    cov_matrix = np.cov(centered_points, rowvar=False)

    # Perform singular value decomposition (SVD) on the covariance matrix
    _, _, v = np.linalg.svd(cov_matrix)

    # Create a rotation matrix from the eigenvectors
    R = v.T

    # Transform points and add back the mean
    transformed_points = (R @ centered_points.T).T + mean

    # Update the point cloud with the transformed points
    pcd.points = o3d.utility.Vector3dVector(transformed_points)

    # Ensure that the major axis points in the desired direction (e.g., Z-axis)
    major_axis = get_major_axis(pcd)
    if major_axis[2] < 0:
        pcd.points = o3d.utility.Vector3dVector(-transformed_points)

    return pcd


def get_major_axis(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Computes the major axis of a point cloud.

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.

    Returns:
        np.ndarray: The major axis vector.
    """
    points = np.asarray(pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    _, _, v = np.linalg.svd(centered_points, full_matrices=False)
    return v[0]
