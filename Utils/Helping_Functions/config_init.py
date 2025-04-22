import configparser
from pathlib import Path

def config_setup():
    config = configparser.ConfigParser()
    config_path = 'config.ini'

    # Lese die vorhandene Konfigurationsdatei, einschließlich der settings-Sektion
    config.read(config_path)
    
    # Lokale Pfade festlegen
    base_dir = Path(__file__).resolve().parent.parent.parent
    cad_dir = base_dir / 'Data' / 'Input' / 'CAD'
    measurements_dir = base_dir / 'Data' / 'Input' / 'Measurements'
    gear_measurements_dir = measurements_dir / 'Gear' / 'Additional Measurements' # added / 'Additional Measurements'
    reany_input_dir = base_dir / 'Data' / 'Input' / 'Reany'
    evaluation_output_dir = base_dir / 'Data' / 'Outpout' # Aaron

    # Dateipfade festlegen
    clamp_measurement_path = measurements_dir / 'Clamp' / 'Clamp_Measurement.txt'
    gear_cad_path = cad_dir / 'Z13_CAD_aligned.txt'

    config['dir'] = {
        'base_dir': base_dir,
        'cad_dir': cad_dir,
        'measurements_dir': measurements_dir,
        'gear_measurements_dir': gear_measurements_dir,
        'reany_input_dir': reany_input_dir,
        'evaluation_output_dir': evaluation_output_dir
    }
    
    config['path'] = {
        'clamp_measurement_path': clamp_measurement_path,
        'gear_cad_path': gear_cad_path
    }

    config['settings'] = {
        'voxel_size_icp_fine': 1,
        'voxel_size_icp_coarse': 1,
        'l_b_delta': 0.1 * 0.75 * 10**-3,
        'z_grenz': 0.75 * 10**-3,
        'z_increment': 0.000125,
        'layers': 6
    }

    # Schreibe die neuen Pfade in die Konfigurationsdatei, ohne die [settings] zu ändern
    with open(config_path, 'w') as configfile:
        config.write(configfile)

