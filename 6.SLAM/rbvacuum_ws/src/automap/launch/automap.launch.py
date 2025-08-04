from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 설정
    turtlebot4_viz_dir = os.path.join(
        get_package_share_directory('turtlebot4_viz'),
        'launch'
    )
    turtlebot4_navigation_dir = os.path.join(
        get_package_share_directory('turtlebot4_navigation'),
        'launch'
    )
    
    # 포함할 launch 파일 정의
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_viz_dir, 'view_robot.launch.py')
        )
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_navigation_dir, 'slam.launch.py')
        )
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_navigation_dir, 'nav2.launch.py')
        )
    )
    
    # # Python 실행 파일 등록
    # mapping = Node(
    #     package='automap',
    #     executable='mapping.py',
    #     name='FrontierExplorer',
    #     output='screen'
    # )
    
    # LaunchDescription에 추가
    return LaunchDescription([
        view_robot_launch,
        LogInfo(condition=None, msg="view_robot.launch.py has been launched."),
        slam_launch,
        LogInfo(condition=None, msg="slam.launch.py has been launched."),
        nav2_launch,
        LogInfo(condition=None, msg="nav2.launch.py has been launched."),
        # mapping
    ])
