import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def get_topside(pc_gear: o3d.geometry.PointCloud, voxel_size: float):
    """
    This function calculates the topmost site of a point cloud based on the normals of the points.

    Args:
        pc_gear (open3d.geometry.PointCloud): The input point cloud.

    Returns:
        float: The z-coordinate of the topmost site of the point cloud.
    """
    # Winkel-Toleranz in Grad
    Tolerance_Angle = 20
    # Umrechnung in Cosinus-Wert
    Tolerance_Angle = -np.cos(Tolerance_Angle / 180 * np.pi)

    # Calculate Gear Normals
    radius_normal = voxel_size * 5
    pc_gear.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # Holen der Normalen der Punktewolke
    Gear_normals = np.asarray(pc_gear.normals)
    print(Gear_normals)

    # Auswahl der Indizes, deren Normalen in x-Richtung unterhalb der Schwelle liegen
    rows_topsite_full = np.where(Gear_normals[:, 0] < Tolerance_Angle)[0]
    topsite = np.asarray(pc_gear.points)[rows_topsite_full]

    # Berechnung von statistischen Maßen für die x-Achse
    topsite_mean_value = np.mean(topsite[:, 0])
    topsite_std_value = np.std(topsite[:, 0])

    # Bestimmung der oberen und unteren Schwellwerte für die x-Achse
    topsite_max = 3 * topsite_std_value + topsite_mean_value
    topsite_min = -3 * topsite_std_value + topsite_mean_value

    # Auswahl der Indizes innerhalb des definierten x-Bereichs
    valid_indices = np.where((topsite[:, 0] < topsite_max) & (topsite[:, 0] > topsite_min))[0]

    # Bestimmung des maximalen x-Werts im gefilterten Bereich
    topsite_value = np.max(topsite[valid_indices, 0])

    return topsite_value


def count_points_in_percentiles(pc):
    # Punktewolke in Numpy umwandeln
    z_values = np.asarray(pc.points)[:, 2]  # Extract Z-axis values
    
    # Berechne Perzentile (1% Schritte)
    min_z, max_z = np.min(z_values), np.max(z_values)
    percentile_bins = np.linspace(min_z, max_z, 101)
    
    # Zähle Punkte in Perzentilen mit Histogramm
    counts, edges = np.histogram(z_values, bins=percentile_bins)
    return edges, counts

def plot_percentile_counts(edges, counts):
    # Plotten der Perzentile
    plt.figure(figsize=(10, 6))
    plt.bar(edges[:-1], counts, width=np.diff(edges), edgecolor='black', align='edge')
    plt.xlabel('Perzentile')
    plt.ylabel('Anzahl an Punkten')
    plt.title('Punkteanzahl in jedem Perzentil der Z-Achse')
    plt.xticks(edges[::10], labels=[f"{edge:.5f}" for edge in edges[::10]])
    plt.grid(True)
    plt.show()
