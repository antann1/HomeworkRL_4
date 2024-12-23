# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,TextSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     slam_params_file = LaunchConfiguration('slam_params_file')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     slam_params_file_arg = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=PathJoinSubstitution(
#             [FindPackageShare("rl_fra2mo_description"), 'config', 'slam.yaml']
#         ),
#         description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
#     )

#     slam_params_file_arg_medium = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=PathJoinSubstitution(
#             [FindPackageShare("rl_fra2mo_description"), 'config', 'slam_medium_values.yaml']
#         ),
#         description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
#     )

#     slam_params_file_arg_high = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=PathJoinSubstitution(
#             [FindPackageShare("rl_fra2mo_description"), 'config', 'slam_high_values.yaml']
#         ),
#         description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
#     )

#     use_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
#     )

#     slam_node = Node(
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node',
#         name='slam',
#         parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
#     )

#     return LaunchDescription([use_sim_time_arg, slam_params_file_arg, slam_node])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Configurazione dei parametri passati tramite terminale
    slam_config_type = LaunchConfiguration('slam_config_type', default='default')  # Tipo di configurazione (default, medium, high)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # Usa il clock di simulazione (Gazebo)

    # Ottieni la directory del pacchetto con ament_index_python
    fra2mo_share_dir = get_package_share_directory('rl_fra2mo_description')

    # Dichiarare gli argomenti per i file di configurazione SLAM
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [fra2mo_share_dir, 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    slam_params_file_arg_medium = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [fra2mo_share_dir, 'config', 'slam_medium_values.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    slam_params_file_arg_high = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [fra2mo_share_dir, 'config', 'slam_high_values.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    # Condizionale per selezionare quale file usare in base al valore dell'argomento 'slam_config_type'
    slam_config_switch = {
        'medium': slam_params_file_arg_medium,
        'high': slam_params_file_arg_high,
        'default': slam_params_file_arg,
    }

    # Dichiarare l'argomento per l'uso del tempo simulato
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )

    # Nodo SLAM: passiamo direttamente il valore del percorso del file di configurazione
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[
            LaunchConfiguration('slam_params_file'),  # Passa il parametro del file di configurazione scelto
            {'use_sim_time': use_sim_time},  # Passa il parametro per il clock di simulazione
        ],
    )

    return LaunchDescription([
        # Passa gli argomenti dichiarati al launch file
        slam_params_file_arg,  # Qui il parametro è dichiarato
        slam_params_file_arg_medium,
        slam_params_file_arg_high,
        use_sim_time_arg,
        slam_node,
    ])