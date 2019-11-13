# linefit_ground_segmentation

Implementation of the ground segmentation algorithm proposed in 
```
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```
The `linefit_ground_segmentation` package contains the ground segmentation library.
A ROS interface is available in `linefit_ground_segmentation_ros` 

The library can be compiled separately from the ROS interface if you're not using ROS.

## Installation

Requires the following dependencies to be installed:

- *catkin_simple* `https://github.com/catkin/catkin_simple.git` 
- *glog_catkin* `https://github.com/ethz-asl/glog_catkin.git`

Compile using your favorite catkin build tool (e.g. `catkin build linefit_ground_segmentation`)

## Launch instructions

The ground segmentation ROS node can be launch by executing `roslaunch linefit_ground_segmentation_ros segmentation.launch`.
Input and output topic names can be specified in the same file.

Getting up and running with your own point cloud source should be as simple as:

1. Change the `input_topic` parameter in `segmentation.launch` to your topic.
2. Adjust the `sensor_height` parameter in `segmentation_params.yaml` to the height where the sensor is mounted on your robot (e.g. KITTI Velodyne: 1.8m)

## Parameter description

Parameters are set in `linefit_ground_segmentation_ros/launch/segmentation_params.yaml`

### Ground Condition
- **sensor_height**  Sensor height above ground.
- **max_dist_to_line**  maximum vertical distance of point to line to be considered ground.
- **max_slope**  Maximum slope of a line.
- **max_fit_error**  Maximum error a point is allowed to have in a line fit.
- **max_start_height**  Maximum height difference between new point and estimated ground height to start a new line.
- **long_threshold**  Distance after which the max_height condition is applied.
- **max_height**  Maximum height difference between line points when they are farther apart than *long_threshold*.
- **line_search_angle**  How far to search in angular direction to find a line. A higher angle helps fill "holes" in the ground segmentation.

### Segmentation

- **r_min**  Distance at which segmentation starts.
- **r_max**  Distance at which segmentation ends.
- **n_bins**  Number of radial bins.
- **n_segments**  Number of angular segments.

### Other

- **n_threads**  Number of threads to use.
- **latch**  Latch output point clouds in ROS node. 
- **visualize** Visualize the segmentation result. **ONLY FOR DEBUGGING.** Do not set true during online operation.
