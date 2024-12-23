from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    explore_config_type = LaunchConfiguration('explore_config_type', default='default')  # Tipo di configurazione (default, medium, high)

    declare_params_file_cmd_low = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore_low_values.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_params_file_cmd_high = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore_high_values.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    explore_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=LaunchConfiguration('slam_params_file'),  # Usa la configurazione determinata dal parametro
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # Condizionale per selezionare quale file usare in base al valore dell'argomento 'slam_config_type'
    explore_config_switch = {
        'low': declare_params_file_cmd_low,
        'high': declare_params_file_cmd_high,
        'default': declare_params_file_cmd,
    }

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_slam.launch.py'])
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items(),
    # )

    slam_config_type = LaunchConfiguration('slam_config_type', default='default')

  
    # Includi il file fra2mo_slam.launch.py e passa i parametri
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_config_type': slam_config_type  # Passa slam_config_type qui
        }.items(),
    )

    
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            explore_config_switch.get(explore_config_type, declare_params_file_cmd),
            declare_use_sim_time_cmd,
            slam_launch,
            nav2_bringup_launch,
            # explore_lite_launch,
        ]
    )