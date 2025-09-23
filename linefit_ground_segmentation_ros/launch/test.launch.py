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
                executable="ground_segmentation_test_node",
                name="ground_segmentation_test",
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
                        "point_cloud_file": PathJoinSubstitution(
                            [
                                linefit_ground_segmentation_share_dir,
                                "doc",
                                "kitti.ply",
                            ]
                        ),
                        "visualize": True,
                        "debug": True,
                    },
                ],
            )
        ]
    )
