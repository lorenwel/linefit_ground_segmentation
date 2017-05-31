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
A ROS interface is contained in `linefit_ground_segmentation_ros` 

The library can be compiled separately from the ROS interface if you're not using ROS.

## Installation

Requires the following dependencies to be installed:

- *catkin_simple* `https://github.com/catkin/catkin_simple.git` 
- *glog_catkin* `https://github.com/ethz-asl/glog_catkin.git`

Compile using your favorite catkin build tool (e.g. `catkin build linefit_ground_segmentation`)

## Launch instructions

The ground segmentation ROS node can be launch by running `segmentation.launch`.
Input and output topic names can be specified in the same file.

## Parameter description

Parameters are set in `launch/segmentation_params.yaml`

### Ground Condition

- **max_slope**  Maximum slope of a line.
- **max_fit_error**  Maximum error a point is allowed to have in a line fit.
- **max_start_height**  Maximum height difference between new point and estimated ground height to start a new line.
- **long_threshold**  Distance after which the max_height condition is applied.
- **max_height**  Maximum height difference between line points when they are farther apart than long_threshold.

### Segmentation

- **r_min**  Distance at which segmentation starts.
- **r_max**  Distance at which segmentation ends.
- **n_bins**  Number of radial bins.
- **n_segments**  Number of angular segments.

### Other

- **n_threads**  Number of threads to use.
- **max_dist_to_line**  Maximum vertical distance of point to line to be considered ground.
