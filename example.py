"""

# Created: 2024-02-14 20:25
# Copyright (C) 2024-now, KTH Royal Institute of Technology
# Author: Qingwen ZHANG  (https://kin-zhang.github.io/)
# License: BSD 3-Clause License

# Description: ground segmentation using linefit

"""
from linefit import ground_seg
import numpy as np
import os, sys, time
BASE_DIR = os.path.abspath(os.path.join( os.path.dirname( __file__ ), '' ))
sys.path.append(BASE_DIR)

if __name__ == "__main__":
    pc_data = np.load(f"{BASE_DIR}/assets/data/kitti0.npy")
    config_path = f"{BASE_DIR}/assets/config.toml"
    view_file = f"{BASE_DIR}/assets/view/center.json"

    print(f"point cloud shape: {pc_data.shape}")
    start_time = time.time()
    if not os.path.exists(config_path):
        print(f"config file {config_path} not found, use default parameters")
        groundseg = ground_seg()
    else:
        groundseg = ground_seg(config_path)
    label = np.array(groundseg.run(pc_data[:,:3].tolist()))
    print(f"point cloud shape: {pc_data[:, :3].shape}, label shape: {label.shape}, ground points: {np.sum(label)}, time: {time.time() - start_time:.3f} s")
    
    from python.utils.o3d_view import MyVisualizer
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_data[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.zeros((pc_data.shape[0], 3)))

    # NOTE: for me to check the height through cloudcomapre by clicking a point. ^v^
    # o3d.io.write_point_cloud(f"{BASE_DIR}/assets/data/kitti0.pcd", pcd) 

    for i in range(pc_data.shape[0]):
        if label[i] == 1:
            pcd.colors[i] = [1, 0, 0] # red

    viz = MyVisualizer(view_file, window_title="Check ground")
    viz.show([pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)])