import launch
import launch_ros.actions


def generate_launch_description():
    xml_path = launch.substitutions.LaunchConfiguration('xml_path')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'xml_path', default_value='',
            description='xml filepath of the database'),
        launch_ros.actions.Node(
            package='vision_info_server',
            node_executable='vision_info_server_node',
            node_name='vision_info_server_node',
            output='screen',
            parameters=[{'xml_path':xml_path}])
    ])
