from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    linefit_ground_segmentation_share_dir = FindPackageShare(
        "linefit_ground_segmentation_ros"
    )

    return LaunchDescription(
        [
            Node(
                package="linefit_ground_segmentation_ros",
                executable="ground_segmentation_node",
                name="ground_segmentation",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            linefit_ground_segmentation_share_dir,
                            "launch",
                            "segmentation_params.yaml",
                        ]
                    ),
                    {
                        "input_topic": "/kitti/velo/pointcloud",
                        "ground_output_topic": "ground_cloud",
                        "obstacle_output_topic": "obstacle_cloud",
                    },
                ],
            )
        ]
    )
