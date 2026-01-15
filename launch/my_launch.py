import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='assignment1.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_bme_gazebo_sensors= get_package_share_directory('assign2')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    

    gazebo_models_path = os.path.join(pkg_bme_gazebo_sensors, 'aruco_boxes_models')

    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    # Safely prepend/append so it works when the var is unset
    prev = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = prev + (os.pathsep + gazebo_models_path if prev else gazebo_models_path)
    # also set classic Gazebo model path variants for compatibility
    prev2 = os.environ.get("GAZEBO_MODEL_PATH", "")
    os.environ["GAZEBO_MODEL_PATH"] = prev2 + (os.pathsep + gazebo_models_path if prev2 else gazebo_models_path)
    prev3 = os.environ.get("GZ_RESOURCE_PATH", "")
    os.environ["GZ_RESOURCE_PATH"] = prev3 + (os.pathsep + gazebo_models_path if prev3 else gazebo_models_path)


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_bme_gazebo_sensors,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
        TextSubstitution(text=' -r -v -v1')],
        'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject
