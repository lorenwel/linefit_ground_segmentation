"""

# Created: 2024-02-14 20:25
# Copyright (C) 2024-now, KTH Royal Institute of Technology
# Author: Qingwen ZHANG  (https://kin-zhang.github.io/)
# License: BSD 3-Clause License

# Description: Testing ground segmentation using open3d

"""
from linefit import ground_seg
import numpy as np
import os, sys
BASE_DIR = os.path.abspath(os.path.join( os.path.dirname( __file__ ), '' ))
sys.path.append(BASE_DIR)

if __name__ == "__main__":
    pc_data = np.load(f"{BASE_DIR}/assets/data/kitti0.npy")
    print(f"point cloud shape: {pc_data.shape}")
    groundseg = ground_seg()
    label = np.array(groundseg.run(pc_data.tolist()))
    print(f"point cloud shape: {pc_data[:, :3].shape}, label shape: {label.shape}, ground points: {np.sum(label)}")

    # import open3d as o3d
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pc_data[:, :3])
    # pcd.colors = o3d.utility.Vector3dVector(np.zeros((pc_data.shape[0], 3)))
    # for i in range(pc_data.shape[0]):
    #     if label[i] == 1:
    #         pcd.colors[i] = [1, 0, 0]
    # o3d.visualization.draw_geometries([pcd])