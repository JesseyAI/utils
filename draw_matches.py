import sys
import numpy as np
import open3d as o3d


def load_point_data_from_file(file_path):
    """
    Load points from a single text file and parse them into a numpy array.
    Each row in the file forms one point (three columns per point).
    """
    points = np.loadtxt(file_path)
    return points

def visualize_cesi(map_points, corners, matches):
    """
    Visualize lines from the point data using a custom visualizer setup.
    Each row in point_data is assumed to contain three points making up one triangle.
    The first set of lines in each file is colored red, others are colored green.
    """

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='Matches', width=2024, height=1800, left=150, top=50)
    vis.get_render_option().background_color = np.array([0, 0, 0])  # Set background to black
    vis.get_render_option().point_size = 1

    def callback1(vis, act, mods):
        if act == 1:
            exit(0)
        else:
            pass
        return True

    map_point_cloud = o3d.geometry.PointCloud()
    map_point_cloud.points = o3d.utility.Vector3dVector(map_points)
    map_point_cloud.paint_uniform_color([1, 1, 1]) #点云颜色

    corner_point_cloud = o3d.geometry.PointCloud()
    corner_point_cloud.points = o3d.utility.Vector3dVector(corners)
    corner_point_cloud.paint_uniform_color([1, 0, 0]) #点云颜色

    vis.add_geometry(map_point_cloud)
    vis.add_geometry(corner_point_cloud)

    for i, row in enumerate(matches):
        vertices = np.reshape(row, (3, 3))  # Reshape row to three points
        lines = [[0, 1], [1, 2], [2, 0]]   # Indices for lines to form a triangle perimeter
        colors = [[1, 0, 0]] if i == 0 else [[0, 1, 0]]  # Red for the first, green for others
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors * len(lines))
        vis.add_geometry(line_set)

    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters('viewpoint.json')
    ctr.convert_from_pinhole_camera_parameters(param)

    vis.register_key_action_callback(32, callback1)
    vis.run()  # Run the visualizer for the current file's geometries

    vis.destroy_window()

        
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script_name.py <matches_dir> <map_dir>")
        sys.exit(1)

    map_dir = sys.argv[1]

    corner_dir = sys.argv[2]

    matches_dir = sys.argv[3]

    matches = load_point_data_from_file(matches_dir)

    map_points = load_point_data_from_file(map_dir)

    corners = load_point_data_from_file(corner_dir)

    visualize_cesi(map_points,corners, matches)
