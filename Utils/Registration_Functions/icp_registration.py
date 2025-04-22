import open3d as o3d
import numpy as np
import configparser


def perform_icp(source_pc, target_pc, mode='fine'):
    """
    Führt je nach gewähltem Modus eine einfache, grobe oder feine ICP-Anpassung durch.

    :param source_pc: Quell-Punktwolke als open3d.geometry.PointCloud
    :param target_pc: Ziel-Punktwolke als open3d.geometry.PointCloud
    :param mode: Modus der ICP-Anpassung ('simple', 'coarse_fine')
    :return: Ein Tuple mit der finalen Transformationsmatrix und Metriken (Fitness, RMSE)
    """
    # Laden der Konfiguration
    config = configparser.ConfigParser()
    config.read('config.ini')

    # Zugriff auf die Einstellungen aus der Sektion 'settings'
    voxel_size_coarse = float(config.get('settings', 'voxel_size_icp_coarse'))
    voxel_size_fine = float(config.get('settings', 'voxel_size_icp_fine'))

    if mode == 'simple' or mode == 'coarse_fine':
        # Einstellungen für die einfache ICP (Standard)
        threshold = voxel_size_coarse * 0.4
        trans_init = np.eye(4)  # Initialisierung mit der Einheitsmatrix
        source_down = source_pc.voxel_down_sample(voxel_size_coarse)
        target_down = target_pc.voxel_down_sample(voxel_size_coarse)

        print("Anwendung der einfachen ICP-Anpassung...")
        result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
        )

        print("---------------------------Transformation ICP results - Coarse")
        print("Transformation Matrix: ", result_icp.transformation)
        print("Fitness: ", result_icp.fitness)
        print("RMSE: ", result_icp.inlier_rmse)

        if mode == 'simple':
            return result_icp.transformation, result_icp.fitness, result_icp.inlier_rmse

    if mode == 'coarse_fine':
        # Zusätzliche Einstellungen für die feine ICP nach der groben Anpassung
        source_down_fine = source_pc.voxel_down_sample(voxel_size_fine)
        target_down_fine = target_pc.voxel_down_sample(voxel_size_fine)
        threshold_fine = voxel_size_fine * 0.4

        print("Anwendung der feinen ICP-Anpassung...")
        result_icp_fine = o3d.pipelines.registration.registration_icp(
            source_down_fine, target_down_fine, threshold_fine, result_icp.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
        )

        print("--------------------------------------------------------------------------------------------")
        print("------------------------Transformation ICP results - Fine")
        print("Transformation Matrix: ", result_icp_fine.transformation)
        print("Fitness: ", result_icp_fine.fitness)
        print("RMSE: ", result_icp_fine.inlier_rmse)

        return result_icp_fine
