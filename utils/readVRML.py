import re
import os
import open3d as o3d
import numpy as np

def parse_vrml_file(file_path):
    with open(file_path, 'r') as file:
        vrml_data = file.read()

    return parse_vrml(vrml_data)

def parse_vrml(vrml_code):
    points = []
    pattern = r"Transform\s*{\s*translation\s*([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)\s*"

    matches = re.findall(pattern, vrml_code)
    for match in matches:
        x, y, z = map(float, match)
        points.append([x, y, z])

    return points

if __name__ == "__main__":
    maps_folder = "/home/ajay/catkin_ws_rmap/coloradar/maps"
    files = sorted(os.listdir(maps_folder))
    mapfiles = [os.path.join(maps_folder, f) for f in files if f.endswith(".wrl")]
    for filepath in mapfiles:
        
        points = parse_vrml_file(filepath)


        points = np.array(points)
        print("Parsed 3D Points:", points.shape)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        output_file = filepath.split(".")[0] + ".ply"
        print(filepath, output_file)
        o3d.io.write_point_cloud(output_file, pcd, True, True)

