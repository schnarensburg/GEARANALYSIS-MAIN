import os
import glob


def get_files_to_process(raw_measurements_dir, evaluation_output_dir):

    # Erfassen aller .txt-Dateien in den Eingabe- und Ausgabeverzeichnissen
    files_in = glob.glob(os.path.join(raw_measurements_dir, '*.txt'))
    files_out = glob.glob(os.path.join(evaluation_output_dir, '*.txt'))

    # Extrahieren der Dateinamen ohne Pfad
    files_in_names = {os.path.basename(file) for file in files_in}
    files_out_names = {os.path.basename(file) for file in files_out}

    # Berechnen der Differenz zwischen den Eingabe- und Ausgabedateien
    files_to_process_names = files_in_names - files_out_names

    # Umwandeln der Dateinamen zurück in vollständige Pfade
    files_to_process = [os.path.join(raw_measurements_dir, filename) for filename in files_to_process_names]

    return files_to_process