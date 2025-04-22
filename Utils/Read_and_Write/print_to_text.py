import numpy as np
import open3d as o3d
from pathlib import Path

def print_to_text(point_cloud, layers, output_dir, filename):

    output_path = Path(output_dir) / f"{filename}.txt"

    with output_path.open('w') as file:
        for i in range(layers):
            file.write(f"Schnitt {i + 1}\n\n")
            file.write("X \t Y \t Z \t\n\n")
            layer = point_cloud[:, :, i]

            # Filter Funktion für ungültige Punkte (x=0, y=0)
            valid_points = layer[(layer[:, 0] != 0) & (layer[:, 1] != 0)]

            for point in valid_points:
                file.write(f"{point[0]:.9f} \t {point[1]:.9f} \t {point[2]:.9f}\n")
            file.write("\n")
