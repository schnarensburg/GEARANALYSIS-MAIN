import open3d as o3d
import numpy as np

from pathlib import Path

from Utils.Read_and_Write.read_data import load_point_cloud_from_file
from Utils.helping_functions import swap_axes
from Utils.helping_functions import get_largest_extent_axis
from Utils.helping_functions import create_global_frame
from Utils.helping_functions import get_frame
from Utils.Registration_Functions.icp_registration import perform_icp


def generate_cad_aligned_to_clamping_system(cad_path, clamp_measurement_path, cad_aligned_path):
    """
    Liest zwei Punktwolken, führt grobe und feine ICP-Anpassungen durch,
    fusioniert sie und speichert die resultierende Punktwolke.

    :param cad_path: Pfad zur Quell-Punktwolke (Datei).
    :param clamp_measurement_path: Pfad zur Ziel-Punktwolke (Datei).
    :param cad_aligned_path: Pfad, wo die fusionierte Punktwolke gespeichert wird.
    """
    # Punktwolken laden
    source_pc = load_point_cloud_from_file(str(cad_path))
    target_pc = load_point_cloud_from_file(str(clamp_measurement_path))
    print("-------Daten erfolgreich geladen.")

    # X-Achse des Spanndornes nach Ausrichtung finden
    max_index , max_axis, max_extent = get_largest_extent_axis(target_pc)

    # Rotationsachsen als x-Achse festlegen
    #source_pc = swap_axes(source_pc, 'z', 'x')
    #target_pc = swap_axes(target_pc, max_axis, 'x')

    # TODO Ausrichtung des KOS der Spanndornpunktewolke

    frame = create_global_frame([source_pc, target_pc])
    o3d.visualization.draw_geometries([source_pc, target_pc, frame])

    # ICP-Ausführen
    # TODO ICP results überpfüen
    result_icp = perform_icp(source_pc, target_pc, mode='coarse_fine')
    print("------ICP durchgeführt.")

    # Anwenden der Transformation auf die ursprüngliche Quell-Punktwolke
    source_pc.transform(result_icp.transformation)

    # Speichern der fusionierten Punktwolke
    print("Speichern der ausgerichteten CAD-Punktwolke...")
    o3d.visualization.draw_geometries([source_pc, get_frame(source_pc)])
    o3d.io.write_point_cloud(str(cad_aligned_path), source_pc)
    print("Ausgerichtete Punktwolke wurde erfolgreich gespeichert.")

