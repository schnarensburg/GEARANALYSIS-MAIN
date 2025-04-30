import open3d as o3d
import numpy as np
import configparser
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import stl as stl
from Utils.Read_and_Write.read_data import load_point_cloud_from_file
from Utils.generate_aligned_CAD_PC import generate_cad_aligned_to_clamping_system
from Utils.Registration_Functions.icp_registration import perform_icp
from Utils.Registration_Functions.ransac_registration import perform_ransac
from Utils.Read_and_Write.print_to_text import print_to_text
from Utils.gear_alignment import gear_alignment, fine_alignment_gear

# TODO : alles einheitlich (englisch und aussagekräftige Variablen)
# TODO : Grafik für generelle Idee etc. 
def cut_layer_generator(measurement_file_path):

    # Laden der Konfiguration
    config: configparser.ConfigParser = configparser.ConfigParser()
    config.read('config.ini')

    # Zugriff auf die Einstellungen aus der Sektion 'settings'
    voxel_size_fine: float = float(config.get('settings', 'voxel_size_icp_fine'))
    voxel_size_coarse: float = float(config.get('settings', 'voxel_size_icp_coarse'))
    l_b_delta: float = float(config.get('settings', 'l_b_delta'))
    z_limit: float = float(config.get('settings', 'z_grenz'))
    z_increment: float = float(config.get('settings', 'z_increment'))
    # layers: int = int(config.get('settings', 'layers'))

    layers = int(z_limit / z_increment)
    cut_layers = np.linspace(0, z_limit, layers + 1)
    cut_layers = np.arange(0, z_limit + z_increment, z_increment)
    
    # Zugriff auf die Dateipfade aus der Sektion 'path'
    reany_input_dir: Path = Path(config.get('dir', 'reany_input_dir'))
    clamp_measurement_path: Path = Path(config.get('path', 'clamp_measurement_path'))
    gear_cad_path: Path = Path(config.get('path', 'gear_cad_path'))

    # Einlesen gemessenen Punktewolke und Spanndorn
    pc_gear: o3d.geometry.PointCloud = load_point_cloud_from_file(measurement_file_path)
    pc_clamp: o3d.geometry.PointCloud = load_point_cloud_from_file(str(clamp_measurement_path))

    # o3d.visualization.draw_geometries([pc_gear, pc_clamp])

    # Ausrichtung der Messung
    pc_gear_cad: o3d.geometry.PointCloud = load_point_cloud_from_file(gear_cad_path)
    pc_gear_aligned, pc_clamp_aligned = gear_alignment(pc_gear, pc_clamp)
    pc_gear_aligned, pc_clamp_aligned = fine_alignment_gear(pc_gear_aligned, pc_clamp_aligned, pc_gear_cad)

    # AUSKLAMMERN VON MAX
    # # Grobausrichtung Messung-CAD mittels RANSAC
    # result_ransac = perform_ransac(pc_gear, pc_gear_aligned)
    # pc_gear.transform(result_ransac.transformation)

    # # Feinausrichtung Messung-CAD mittels ICP
    # result_icp = perform_icp(pc_gear, pc_gear_aligned, mode='coarse_fine')
    # pc_gear.transform(result_icp.transformation)

    # # Ergebnis visualisieren
    # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.002, origin=[0,0,0])
    # o3d.visualization.draw_geometries([pc_clamp_aligned, pc_gear_aligned, axis])

    gear_aligned = np.asarray(pc_gear_aligned.points)
    clamp_aligned = np.asarray(pc_clamp_aligned.points)

    # # Speichern der ausgerichteten Punktewolken vor dem CLG
    # aligned_path = Path(measurement_file_path).stem
    # aligned_output_dir = Path(config.get('dir','reany_input_dir')).parent / 'Aligned'
    # aligned_output_dir.mkdir(parents=True, exist_ok=True)
    # np.savetxt(aligned_output_dir / str(aligned_path + '_aligned'), gear_aligned)
    # complete = np.concatenate((gear_aligned, clamp_aligned), axis=0)
    # np.savetxt(aligned_output_dir / str(aligned_path + '_aligned_with_clamp'), complete)

    # Filter points innerhalb z_grenz beginnend bei 0
    mask = (gear_aligned[:, 2] >= 0) & (gear_aligned[:, 2] <= z_limit)
    gearing = gear_aligned[mask]
    
    # # Visualisiere Schnittbereich
    # o3d_gearing = o3d.geometry.PointCloud()
    # o3d_gearing.points = o3d.utility.Vector3dVector(gearing)
    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.002, origin=[0, 0, 0])
    # o3d.visualization.draw_geometries([
    #     o3d_gearing,
    #     coordinate_frame,
    # ])

    # Sortieren der Punkte nach Winkel
    angles = np.arctan2(gearing[:, 0], gearing[:, 1]) # Winkel vom Ursprung aus gesehen
    angles = np.mod(angles, 2* np.pi)
    sorted_indices = np.argsort(angles)
    gearing = gearing[sorted_indices]

    # Initialisiere Liste für die Schichten
    gearing_cut_layers = []

    for i in range(layers):
        mask_layer = (gearing[:, 2] >= cut_layers[i]) & (gearing[:, 2] <= cut_layers[i + 1])
        selected_points = gearing[mask_layer]
        selected_points[:, 2] = cut_layers[i]  # Z-Wert auf Schichthöhe setzen
        gearing_cut_layers.append(selected_points)

    # Berechnen der maximalen Länge der Schichten
    length_max = max(layer.shape[0] for layer in gearing_cut_layers)

    gearing_cut_layers_print_array = np.zeros((length_max, 3, len(gearing_cut_layers)))

    for i in range(len(gearing_cut_layers)):
        selected_points = gearing_cut_layers[i]
        sum_layer_points = selected_points.shape[0]
        gearing_cut_layers_print_array[:sum_layer_points, :, i] = selected_points[:, :3]
        if sum_layer_points < length_max:
            gearing_cut_layers_print_array[sum_layer_points:, :, i] = 0

    # Visualisieren der Schichten
    # visualize_layers(gearing_cut_layers_print_array)

    # Schreiben der Schichten in eine spezielle Textdatei
    reany_input_dir.mkdir(parents=True, exist_ok=True)    
    print_to_text(gearing_cut_layers_print_array, layers, reany_input_dir, measurement_file_path.stem + '_cut_layers')
    
    # # STL
    # stl_path = Path(measurement_file_path).stem
    # stl_output_dir = Path(config.get('dir','reany_input_dir')).parent / 'STL'
    # stl_output_dir.mkdir(parents=True, exist_ok=True)
    # tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pc_gear_aligned)
    # alpha = np.logspace(np.log10(0.00010), np.log10(0.00010), num=1)
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    #     pc_gear_aligned, alpha, tetra_mesh, pt_map)
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    # o3d.io.write_triangle_mesh(str(stl_output_dir / str(stl_path + '_STL.stl')), mesh)

    # AUSKLAMMERN VON MAX
    # # --------------------------------------------> Aktueller Stand
    # topsite_value = get_topside(pc_gear, voxel_size_coarse)
    # pc_gear.points = o3d.utility.Vector3dVector(np.asarray(pc_gear.points) - np.array([0, 0, topsite_value]))

    # if np.mean(np.asarray(pt_gear.points)[:, 2]) < 0:
    #     pt_gear.rotate(o3d.geometry.get_rotation_matrix_from_xyz([np.pi, 0, 0]))

    # valid_indices = np.where((np.asarray(pt_gear.points)[:, 2] >= L_b_delta / 2) & (np.asarray(pt_gear.points)[:, 2] <= z_grenz))[0]
    # pt_gear = pt_gear.select_by_index(valid_indices)
    # pt_gear.points = o3d.utility.Vector3dVector(np.asarray(pt_gear.points) - np.array([0, 0, np.asarray(pt_gear.points)[:,2].min()]))

    # for layer in range(layers):
    #     layer_indices = np.where((np.asarray(pt_gear.points)[:, 2] >= cut_layers[layer]) & (np.asarray(pt_gear.points)[:, 2] < cut_layers[layer + 1]))[0]
    #     pts = np.asarray(pt_gear.points)[layer_indices]
    #     pts[:, 2] = cut_layers[layer]
    #     if len(layer_indices) > 0:
    #         pt_gear.points = o3d.utility.Vector3dVector(pts)

    # # Visualisieren der Punktwolke
    # o3d.visualization.draw_geometries([pt_gear])
    

def visualize_layers(gearing_cut_layers_print_array):

    geometries = []
    layers = gearing_cut_layers_print_array.shape[2]
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.002, origin=[0, 0, 0])

    for i in range(layers):
        layer_points = gearing_cut_layers_print_array[:, :, i]
        valid_points = layer_points[np.any(layer_points != 0, axis=1)]
        
        if valid_points.size == 0:
            continue
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(valid_points)
        
        # Farben von blau nach rot
        color = plt.cm.jet(i / layers)[:3]
        colors = np.tile(color, (valid_points.shape[0], 1))
        
        pcd.colors = o3d.utility.Vector3dVector(colors)
        geometries.append(pcd)
    
    o3d.visualization.draw_geometries(geometries + [coordinate_frame])

