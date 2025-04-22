import open3d as o3d
import numpy as np
from pathlib import Path

def load_point_cloud_from_file(filepath):
    """
    Liest eine Datei (TXT oder PLY) und konvertiert sie in eine Open3D Point Cloud.
    :param filepath: Pfad zur Datei.
    :return: Open3D Point Cloud Objekt.
    """

    filepath = Path(filepath)
    # Erkennen des Dateiformats anhand der Dateiendung
    extension = filepath.suffix[1:].lower()

    # Initialisierung der Punktewolke
    point_cloud = o3d.geometry.PointCloud()

    points = []
    # Case Handling
    if extension == 'txt' or extension == 'xyz':
        # TXT oder XYZ Datei
        with open(filepath, 'r') as file:
            for line in file:
                if line.strip() and not line.startswith("GPoint3DVector"):
                    parts = line.strip().split()
                    if len(parts) == 3:
                        points.append([float(part) for part in parts])
        # Zuweisung der Punkte zur Punktwolke
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
    elif extension == 'ply':
        # Direktes Laden einer PLY-Datei
        point_cloud = o3d.io.read_point_cloud(filepath)
    else:
        raise ValueError("Unsupported file format. Only .txt and .ply files are supported.")

    return point_cloud
