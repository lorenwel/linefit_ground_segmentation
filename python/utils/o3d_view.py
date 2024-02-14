'''
# Created: 2023-1-26 16:38
# Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
# Author: Kin ZHANG  (https://kin-zhang.github.io/)

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

This file is for open3d view control set from view_file, which should be json
1. use normal way to open any geometry and set view by mouse you want
2. `CTRL+C` it will copy the view detail at this moment.
3. `CTRL+V` to json file, you can create new one
4. give the json file path
Check this part: http://www.open3d.org/docs/release/tutorial/visualization/visualization.html#Store-view-point

Test if you want by run this script

Then press 'V' on keyboard, will set from json

'''

import open3d as o3d
import json
import os, sys
from typing import List
BASE_DIR = os.path.abspath(os.path.join( os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)

def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip("#")
    return tuple(int(hex_color[i:i + 2], 16) / 255.0 for i in (0, 2, 4))

color_map_hex = ['#a6cee3','#1f78b4','#b2df8a','#33a02c','#fb9a99','#e31a1c','#fdbf6f','#ff7f00','#cab2d6','#6a3d9a','#ffff99','#b15928',\
                 '#8dd3c7','#ffffb3','#bebada','#fb8072','#80b1d3','#fdb462','#b3de69','#fccde5','#d9d9d9','#bc80bd','#ccebc5','#ffed6f']
color_map = [hex_to_rgb(color) for color in color_map_hex]

class ViewControl:
    def __init__(self, vctrl: o3d.visualization.ViewControl, view_file=None):
        self.vctrl = vctrl
        self.params = None
        if view_file is not None:
            print(f"Init with view_file from: {view_file}")
            self.parase_file(view_file)
            self.set_param()
        else:
            print("Init without view_file")
    def read_viewTfile(self, view_file):
        self.parase_file(view_file)
        self.set_param()
    def save_viewTfile(self, view_file):
        return
    def parase_file(self, view_file):
        if(os.path.exists(view_file)):
            with open((view_file)) as user_file:
                file_contents = user_file.read()
                self.params = json.loads(file_contents)
        else:
            print(f"\033[91mDidn't find the file, please check it again: {view_file} \033[0m")
            print(f"NOTE: If you still have this error, please give the absulote path for view_file")
            sys.exit()

    def set_param(self):
        self.vctrl.change_field_of_view(self.params['trajectory'][0]['field_of_view'])
        self.vctrl.set_front(self.params['trajectory'][0]['front'])
        self.vctrl.set_lookat(self.params['trajectory'][0]['lookat'])
        self.vctrl.set_up(self.params['trajectory'][0]['up'])
        self.vctrl.set_zoom(self.params['trajectory'][0]['zoom'])

class MyVisualizer:
    def __init__(self, view_file=None, window_title="Default"):
        self.params = None
        self.viz = o3d.visualization.VisualizerWithKeyCallback()
        self.viz.create_window(window_name=window_title)
        self.o3d_vctrl = ViewControl(self.viz.get_view_control(), view_file=view_file)
        self.view_file = view_file
        
    def show(self, assets: List):
        self.viz.clear_geometries()

        for asset in assets:
            self.viz.add_geometry(asset)
        if self.view_file is not None:
            self.o3d_vctrl.read_viewTfile(self.view_file)

        self.viz.update_renderer()
        self.viz.poll_events()
        self.viz.run()
        self.viz.destroy_window()

if __name__ == "__main__":
    view_json_file = f"{BASE_DIR}/data/o3d_view/default_test.json"
    sample_ply_data = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.path)
    # 1. define
    viz = o3d.visualization.VisualizerWithKeyCallback()
    # 2. create
    viz.create_window(window_name="TEST ON Change View point through JSON, Press V Please")
    # 3. add geometry
    viz.add_geometry(pcd)
    # 4. get control !!! must step by step
    ctr = viz.get_view_control()

    o3d_vctrl = ViewControl(ctr)

    def set_view(viz):
        #Your update routine
        o3d_vctrl.read_viewTfile(view_json_file)
        viz.update_renderer()
        viz.poll_events()
        viz.run()

    viz.register_key_callback(ord('V'), set_view)
    viz.run()
    viz.destroy_window()
    print("\033[92mAll o3d_view codes run successfully, Close now..\033[0m See you!")

    # or:
    # viz = MyVisualizer(view_file, window_title="Check Pose")
    # viz.show([*pcds, *draw_tfs])