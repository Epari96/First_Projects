import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = "car_tutorial"

    # robot_state_publisher
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, "urdf", "car.xacro") #car turtlebot3_burger
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": False}
    world_file = os.path.join(pkg_path, "world", "with_robot.world")
    
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # rviz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "src/car_tutorial/config/car.rviz"],
    )

    # driver
    driver = Node(
        package="car_tutorial",
        executable="driver",
        output="screen",
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # 로봇 스폰
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "car_robot"
        ],
        output="screen",
    )

    odom = Node(
        package="car_tutorial",
        executable= "odom",
        output="screen",
    )

    return LaunchDescription(
        [
            rsp,
            #rsp3,
            rviz,
            gazebo,
            spawn_entity,
            driver,
            odom,
        ]
    )
