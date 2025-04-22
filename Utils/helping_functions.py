import open3d as o3d
import numpy as np
from typing import List, Optional, Dict, Tuple, Union


def get_major_axis(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """
    This function calculates the major axis of a point cloud.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.

    Returns:
        numpy.ndarray: The major axis of the point cloud.
    """
    points = np.asarray(pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    u, s, v = np.linalg.svd(centered_points, full_matrices=False)
    return v[0]


def align_axes(source_axis, target_axis):
    """
    This function aligns two axes using the Rodrigues' rotation formula.

    Args:
        source_axis (numpy.ndarray): The source axis.
        target_axis (numpy.ndarray): The target axis.

    Returns:
        numpy.ndarray: The rotation matrix for aligning the source axis to the target axis.
    """
    v = np.cross(source_axis, target_axis)
    c = np.dot(source_axis, target_axis)
    s = np.linalg.norm(v)

    # Skew-symmetric cross-product matrix von v
    Vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    # Rotationsmatrix R
    R = np.eye(3) + Vx + Vx @ Vx * ((1 - c) / (s ** 2))
    return R


def apply_transformation(pcd, R):
    """
    This function applies a transformation to a point cloud.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        R (numpy.ndarray): The rotation matrix.

    Returns:
        open3d.geometry.PointCloud: The transformed point cloud.
    """
    points = np.asarray(pcd.points)
    rotated_points = R @ points.T  # Anwendung der Rotationsmatrix
    pcd.points = o3d.utility.Vector3dVector(rotated_points.T)
    return pcd


def swap_axes(pcd: o3d.geometry.PointCloud, axis1, axis2) -> o3d.geometry.PointCloud:
    """
    Vertauscht zwei Achsen in einer Punktwolke.

    Args:
    pcd (o3d.geometry.PointCloud): Die zu transformierende Punktwolke.
    axis1 (str): Die erste Achse, die vertauscht werden soll ('x', 'y' oder 'z').
    axis2 (str): Die zweite Achse, die vertauscht werden soll ('x', 'y' oder 'z').

    Returns:
    o3d.geometry.PointCloud: Die transformierte Punktwolke.
    """
    # Achsenindex basierend auf den Buchstaben 'x', 'y', 'z'
    axes_dict = {'x': 0, 'y': 1, 'z': 2}
    index1 = axes_dict[axis1]
    index2 = axes_dict[axis2]

    # Sicherstellen, dass gültige Achsen angegeben wurden
    if index1 == index2:
        return pcd

    # Rotationsmatrix initialisieren als Einheitsmatrix
    R = np.eye(3)

    # Achsen in der Matrix vertauschen
    R[index1, index1], R[index2, index2] = 0, 0  # Diagonalelemente auf 0 setzen
    R[index1, index2], R[index2, index1] = 1, 1  # Off-diagonalelemente auf 1 setzen, um Achsen zu vertauschen

    # Punkte der Punktwolke transformieren
    points = np.asarray(pcd.points)
    transformed_points = R @ points.T

    # Transformierte Punkte zurück in die Punktwolke einfügen
    pcd.points = o3d.utility.Vector3dVector(transformed_points.T)
    return pcd


def get_largest_extent_axis(pcd):
    """
    This function calculates the largest extent axis of a point cloud.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.

    Returns:
        tuple: A tuple containing the index of the largest extent axis,
               the name of the axis ('x', 'y', or 'z'), and the extent of the axis.
    """
    bbox = pcd.get_axis_aligned_bounding_box()
    extents = np.array(bbox.get_extent())
    max_index = np.argmax(extents)  # Index of the largest extent
    axes_dict = {0: 'x', 1: 'y', 2: 'z'}
    max_axis = axes_dict[max_index]  # Mapping of the index to the axis
    return max_index, max_axis, extents[max_index]


def get_frame(target_pc):
    """
    Generates a coordinate frame for a given point cloud.

    Args:
        target_pc (open3d.geometry.PointCloud): The input point cloud.

    Returns:
        open3d.geometry.TriangleMesh: A coordinate frame mesh.
    """
    bbox = target_pc.get_axis_aligned_bounding_box()
    max_extent = max(bbox.get_extent())
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=max_extent * 0.1,
                                                                         origin=bbox.get_center())
    return coordinate_frame


def create_global_frame(point_clouds, scale_factor=0.1):
    """
    This function creates a global frame for a set of point clouds.

    Args:
        point_clouds (list): A list of point clouds.
        scale_factor (float, optional): A scale factor for the size of the frame. Defaults to 0.1.

    Returns:
        open3d.geometry.TriangleMesh: A coordinate frame mesh.
    """

    # Extrahiert alle Punkte aus den gegebenen Punktwolken
    all_points = np.concatenate([pc.points for pc in point_clouds], axis=0)

    # Berechnet das Zentrum aller Punktwolken
    center = all_points.mean(axis=0)

    # Berechnet die Bounding Box aller Punktwolken
    min_bound = all_points.min(axis=0)
    max_bound = all_points.max(axis=0)

    # Bestimmt die größte Dimension der Bounding Box
    max_size = np.max(max_bound - min_bound)

    # Berechnet die Skala des Koordinatensystems basierend auf dem größten Maß
    scale = max_size * scale_factor

    # Erstellt das Koordinatensystem am Zentrum der Punktwolken mit der berechneten Skala
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale, origin=center)
    return frame



