from setuptools import setup

package_name = "robot_autonomy"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/fake_laser_scan.launch.py", "launch/slam_fake_laser.launch.py"],
        ),
        ("share/" + package_name + "/config", ["config/slam_toolbox_params.yaml"]),
        (
            "share/" + package_name + "/rviz",
            ["rviz/fake_laser_scan.rviz", "rviz/slam_fake_laser.rviz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot Autonomy",
    maintainer_email="user@example.com",
    description="Robot autonomy utilities including a fake LaserScan publisher.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_laser_scan_publisher = robot_autonomy.fake_laser_scan_publisher:main",
        ],
    },
)
