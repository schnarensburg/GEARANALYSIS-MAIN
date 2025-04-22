import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R


def find_rotation_axis(pc_clamp):
    """
    Bestimmt die Rotationsachse einer Klammer, indem PCA angewendet wird, um die Hauptkomponente zu identifizieren
    :param pc_clamp: Ein Numpy-Array, das die Klammer darstellt
    :return: Ein Numpy-Array, das die Rotationsachse darstellt
    """

    # Zentriere die Punkte, indem der Mittelwert subtrahiert wird
    points_mean = np.mean(pc_clamp, axis=0)
    points_centered = pc_clamp - points_mean

    # Führe PCA durch
    pca = PCA(n_components=3)
    pca.fit(points_centered)

    # Extrahiere die Hauptkomponente mit der höchsten Varianz
    if pca.components_[0][1] < 0:
        rotation_axis = -pca.components_[0]
    else:
        rotation_axis = pca.components_[0]
    return rotation_axis



def rotation_matrix_z_axis_alignment(vector):
    """
    Berechnet eine Rotationsmatrix, die einen gegebenen Vektor mit der Z-Achse ausrichtet
    :param vector: Ein Numpy-Array, das den 3D-Vektor darstellt, der mit der Z-Achse ausgerichtet werden soll
    :return: Ein 3x3 Numpy-Array, das die Rotationsmatrix darstellt
    """
    
    # Normalisiere den Vektor
    vector = vector / np.linalg.norm(vector)

    # Definiere die Z-Achse
    z_axis = np.array([0, 0, 1])
        
    # Berechne die Rotationsachse und den Winkel
    axis = np.cross(vector, z_axis)
    angle = np.arccos(np.dot(vector, z_axis))

    # Berechne die Rotationsmatrix
    if np.linalg.norm(axis) < 1e-10:  # Überprüfe, ob der Vektor parallel zur Z-Achse ist
        if vector[2] < 0:
            return np.diag([1, 1, -1])  # 180-Grad-Drehung um eine beliebige senkrechte Achse
        return np.eye(3)  # Keine Drehung erforderlich

    # Normalisiere die Rotationsachse
    axis = axis / np.linalg.norm(axis)

    # Rodrigues' Rotationsformel
    k = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    rotation_matrix = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)

    return rotation_matrix


def rotation_matrix_y_axis_alignment(vector):
    """
    Berechnet eine Rotationsmatrix, um einen gegebenen Vektor durch Drehung um die Z-Achse mit der Y-Achse auszurichten
    :param vector: Ein Numpy-Array, das den 3D-Vektor darstellt, der mit der Y-Achse ausgerichtet werden soll
    :return: Ein 3x3 Numpy-Array, das die Rotationsmatrix darstellt
    """

    # Normalisiere den Vektor
    x, y = vector[0], vector[1]

    # Berechne den Winkel des Vektors mit der Y-Achse
    theta = np.arctan2(x, y)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)

    # Definiere die Rotationsmatrix für die Drehung um die Z-Achse
    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])

    return rotation_matrix


def gear_alignment(pc_gear, pc_clamp):
    """
    Richtet eine Zahnrad-Punktwolke mit einer angegebenen Rotationsachse und der Y-Achse aus
    :param pc_gear: Eine Open3D Punktewolke, die das Zahnrad darstellt
    :param pc_clamp: Eine Open3d Punktewolke, die den Spanndorn darstellt
    :return: Eine Open3D Punktewolke, die das ausgerichtete Zahnrad darstellt
    """
    
    # Konvertiere Open3D Punktwolken in Numpy-Arrays
    pc_gear_np = np.asarray(pc_gear.points)

    pc_clamp_np = np.asarray(pc_clamp.points)
    rotation_axis = find_rotation_axis(pc_clamp_np)

    # Berechne die Rotationsmatrix, um die Rotationsachse mit der Z-Achse auszurichten
    rotation_matrix_z = rotation_matrix_z_axis_alignment(rotation_axis)

    # Transformiere die Rotationsachse zur Überprüfung
    rotation_axis_rotated = rotation_matrix_z @ rotation_axis

    # Wende die Rotationsmatrix auf die Zahnrad-Punktwolke an
    pc_gear_aligned_np = np.dot(pc_gear_np, rotation_matrix_z.T)
    pc_clamp_aligned_np = np.dot(pc_clamp_np, rotation_matrix_z.T)

    # Finde den entferntesten Punkt auf dem Zahnrad
    distances_squared = np.sum(pc_gear_aligned_np[:, :2] ** 2, axis=1)
    farthest_point_index = np.argmax(distances_squared)
    
    # Extract the coordinates of the farthest point
    farthest_point = pc_gear_aligned_np[farthest_point_index]

    # Compute the rotation matrix to align the farthest point with the Y-axis
    farthest_point_2D = np.array([farthest_point[0], farthest_point[1], 0])
    rotation_matrix_y = rotation_matrix_y_axis_alignment(farthest_point_2D)

    # Transformiere den entferntesten Punkt zur Überprüfung
    farthest_point_rotated = np.dot(farthest_point, rotation_matrix_y.T)

    # Wende die Rotationsmatrix auf die Zahnrad-Punktwolke an
    pc_gear_aligned_np = np.dot(pc_gear_aligned_np, rotation_matrix_y.T)
    pc_clamp_aligned_np = np.dot(pc_clamp_aligned_np, rotation_matrix_y.T)

    # Bestimme die Oberseite mithilfe der Perzentilmethode
    z_values = np.asarray(pc_gear_aligned_np)[:, 2]
    min_z, max_z = np.min(z_values), np.max(z_values)
    percentile_bins = np.linspace(min_z, max_z, 101)

    # Berechne die Anzahl der Punkte in jedem Perzentil
    counts, percentiles = np.histogram(z_values, bins=percentile_bins)
    peak_index = np.argmax(counts)
    z_value_to_subtract = percentiles[peak_index+1]

    # Subtrahiere z_value_to_subtract von den Z-Werten von pc_gear_aligned_np
    pc_gear_aligned_np[:, 2] -= z_value_to_subtract
    pc_clamp_aligned_np[:, 2] -= z_value_to_subtract

    # Subtrahiere z_value_to_subtract von den Z-Werten von farthest_point_rotated
    farthest_point_rotated[2] -= z_value_to_subtract

    # print("Rotation Matrix to Align to Z-axis:\n", rotation_matrix_to_z)
    # print("-----\n")
    # print("Transformed Rotation Axis (should be along Z-axis):\n", rotation_axis_rotated)
    # print("-----\n")
    # print("Farthest Point:\n", farthest_point)
    # print("-----\n")
    # print("Rotation Matrix to Align to Y-axis:\n", rotation_matrix_to_y)
    # print("-----\n")
    # print("Transformed Nearest Point (should be along Y-axis, disregard Z-axis):\n", farthest_point_rotated)
    # print("-----\n")
    # print("Z-Value to subtract" , z_value_to_subtract)
    # print("-----\n")

    # Erstelle Open3D-Punktwolken
    pc_gear_aligned_o3d = o3d.geometry.PointCloud()
    pc_gear_aligned_o3d.points = o3d.utility.Vector3dVector(pc_gear_aligned_np)
    pc_clamp_aligned_o3d = o3d.geometry.PointCloud()
    pc_clamp_aligned_o3d.points = o3d.utility.Vector3dVector(pc_clamp_aligned_np)

    # sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.00005)
    # sphere.translate(farthest_point_rotated)
    # sphere.paint_uniform_color([1, 0, 0])  # Rote Farbe

    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.002, origin=[0, 0, 0])
    # o3d.visualization.draw_geometries([
    #     pc_gear_aligned_o3d,
    #     sphere,
    #     coordinate_frame,
    # ])
    
    return pc_gear_aligned_o3d, pc_clamp_aligned_o3d


def compute_threshold(pc_gear, pc_gear_cad, factor=0.01):
    """
    Berechnet einen angemessenen Schwellenwert für ICP basierend auf der Größe des Begrenzungsrahmens
    :param pc_gear: Ein Numpy-Array, das die Zahnrad-Punktwolke darstellt
    :param pc_gear_cad: Ein Numpy-Array, das die CAD-Modell-Punktwolke darstellt
    :param factor: Ein Skalierungsfaktor für die Diagonallänge des Begrenzungsrahmens
    :return: Ein Float, der den Schwellenwert darstellt
    """

    # Berechne die Begrenzungsrahmen
    bbox_gear = pc_gear.get_axis_aligned_bounding_box()
    bbox_gear_cad = pc_gear_cad.get_axis_aligned_bounding_box()
    
    # Berechne die Diagonallängen der Begrenzungsrahmen
    diagonal_gear = np.linalg.norm(bbox_gear.get_max_bound() - bbox_gear.get_min_bound())
    diagonal_gear_cad = np.linalg.norm(bbox_gear_cad.get_max_bound() - bbox_gear_cad.get_min_bound())
    
    # Setze den Schwellenwert als Bruchteil der durchschnittlichen Diagonallänge
    threshold = factor * (diagonal_gear + diagonal_gear_cad) / 2
    
    return threshold


def fine_alignment_gear(pc_gear, pc_clamp, pc_gear_cad):
    """
    Richtet eine Zahnrad-Punktwolke an einem CAD-Modell aus
    :param pc_gear: Eine Open3D Punktewolke, die das Zahnrad darstellt
    :param pc_clamp: Eine Open3d Punktewolke, die den Spanndorn darstellt
    :param pc_gear_cad: Eine Open3D Punktewolke, die das CAD-Modell darstellt
    :return: Eine Open3D Punktewolke, die das ausgerichtete Zahnrad darstellt
    """

    # Schätze die Normalen (optional, aber empfohlen für ICP)
    pc_gear.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pc_gear_cad.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Wende den ICP-Algorithmus an
    threshold = compute_threshold(pc_gear, pc_gear_cad, factor=0.01)
    
    icp_result = o3d.pipelines.registration.registration_icp(
        pc_gear, pc_gear_cad, threshold,
        np.eye(4),  # Initiale Transformationsmatrix (Identität)
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # Transformation bis auf die Z-Achse begrenzen
    constrained_transformation = np.eye(4)
    constrained_transformation[:2, 3] = icp_result.transformation[:2, 3]
    constrained_transformation[:2,:2] = icp_result.transformation[:2,:2]

    # Anwenden der Transformation auf eine neue Punktwolke
    pc_gear_aligned = o3d.geometry.PointCloud()
    pc_gear_aligned.points = pc_gear.points
    pc_gear_aligned = pc_gear_aligned.transform(constrained_transformation)

    pc_clamp_aligned = o3d.geometry.PointCloud()
    pc_clamp_aligned.points = pc_clamp.points
    pc_clamp_aligned = pc_clamp_aligned.transform(constrained_transformation)


    # # Open3D Punktewolken zur Visualisierung
    # pc_gear.paint_uniform_color([0, 1, 0])

    # pc_gear_aligned.paint_uniform_color([0, 0, 1])

    # pc_gear_cad.paint_uniform_color([1, 0, 0])

    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.002, origin=[0, 0, 0])
    # o3d.visualization.draw_geometries([
    #     # pc_gear,
    #     pc_gear_cad,
    #     pc_gear_aligned,
    #     coordinate_frame,
    # ])

    return pc_gear_aligned, pc_clamp_aligned
