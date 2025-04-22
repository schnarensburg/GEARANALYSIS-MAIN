import glob
import time
import os
from pathlib import Path
import configparser

from Utils.Helping_Functions.config_init import config_setup
from Utils.Helping_Functions.get_files_to_process import get_files_to_process
from Utils.cut_layer_generator import cut_layer_generator


def main():
    # Laden der Konfiguration
    config_setup()

    # Laden der Konfiguration
    config = configparser.ConfigParser()
    config.read('config.ini')

    # Zugriff auf die Pfade aus der Sektion 'dir'
    gear_measurements_dir = Path(config.get('dir', 'gear_measurements_dir'))
    evaluation_output_dir = Path(config.get('dir', 'evaluation_output_dir'))

    filepaths = list(gear_measurements_dir.glob("*.txt"))
    total_files = len(filepaths)    

    for i, filepath in enumerate(filepaths):
        cut_layer_generator(filepath)
        # Fortschrittsanzeige
        print(f"\rCut Layer Generator: [{'#' * int(30 * (i + 1) / total_files):<30}] {i + 1}/{total_files} files processed", end='', flush=True)
    print()

    # AUSKLAMMERN VON MAX
    # file_count = 0
    # while True:
    #     # Erfassen aller .txt-Dateien in den Verzeichnissen
    #     files_in = glob.glob(os.path.join(gear_measurements_dir, '*.txt'))
    #     number_files_in = len(files_in)
    #     files_out = glob.glob(os.path.join(evaluation_output_dir, '*.txt'))
    #     number_files_out = len(files_out)

    #     if number_files_in > number_files_out:
    #         files_to_process = get_files_to_process(gear_measurements_dir, evaluation_output_dir)

    #         for file in files_to_process:
    #             cut_layer_generator(file)
    #             print(f"Messung {file_count} von {files_to_process} ausgewertet")
    #             file_count += 1

    #         file_count = 1  # Zurücksetzen des Zählers nach der Verarbeitung

    #     time.sleep(10)  # Verzögerung zur Simulation eines Warte-Balkens oder für die systemlast-Regulierung


if __name__ == '__main__':
    main()
