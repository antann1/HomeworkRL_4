
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


 

def generate_launch_description():
    

    other_launch_file_gazebo = PathJoinSubstitution(
        [FindPackageShare("rl_fra2mo_description"), "launch", "gazebo_fra2mo.launch.py"]
    )

    
    include_other_launch_gazebo= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file_gazebo)
    )

    other_launch_file_rviz= PathJoinSubstitution(
        [FindPackageShare("rl_fra2mo_description"), "launch", "display_fra2mo.launch.py"]
    )

    
    include_other_launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file_rviz)
    )

    
    return LaunchDescription([
        include_other_launch_gazebo, 
        include_other_launch_rviz

    ])
