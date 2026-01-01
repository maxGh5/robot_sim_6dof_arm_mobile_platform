from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_autonomy"), "rviz", "fake_laser_scan.rviz"]
    )

    fake_scan_node = Node(
        package="robot_autonomy",
        executable="fake_laser_scan_publisher",
        name="fake_laser_scan_publisher",
        output="screen",
        parameters=[
            {
                "frame_id": "laser_front",
                "scan_topic": "/scan",
                "scan_rate_hz": 5.0,
                "num_points": 360,
                "range_m": 2.0,
            }
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.2",
            "0.0",
            "0.1",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "laser_front",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([fake_scan_node, static_tf, rviz])
