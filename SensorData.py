
# import open3d as o3d
import numpy as np

class SensorData:

    def __init__(self):
        self.data = []
        pass

    def add_data(self, new_data):
        self.data.append(new_data[new_data[:,2] > -2.0])

    # def display_frame(self, i):
    #     print("Showing lidar")
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(self.data[15])
    #     o3d.visualization.draw_geometries([pcd])