import open3d as o3d
import configparser


def perform_ransac(source_pc, target_pc):
    """
    Registriert zwei Punktewolken zuerst mit RANSAC und anschließend mit ICP zur Feinjustierung.
    :param source_pc: Die Quell-Punktewolke.
    :param target_pc: Die Ziel-Punktewolke.
    :return: Transformierte Quell-Punktewolke, RANSAC-Transformation.
    """
    print("RanSAC-Registrierung...")
    # Lade Konfiguration
    config = configparser.ConfigParser()
    config.read('config.ini')

    voxel_size = float(config.get('settings', 'voxel_size_icp_coarse'))

    # Voreinstellungen
    radius_feature = voxel_size * 5
    radius_normal = voxel_size * 2
    threshold = voxel_size * 1.5

    # Vorbereitung der Punktewolken
    source_down = source_pc.voxel_down_sample(voxel_size)
    target_down = target_pc.voxel_down_sample(voxel_size)

    # Schätzung der Normalen
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # Berechnung der FPFH-Features
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    # RANSAC-Registrierung
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source=source_down,
        target=target_down,
        source_feature=source_fpfh,
        target_feature=target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold=threshold),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(similarity_threshold=0.1)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=1000000, confidence=0.99)
    )

    return ransac_result
