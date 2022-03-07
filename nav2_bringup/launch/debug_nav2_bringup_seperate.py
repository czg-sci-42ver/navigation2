from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_composition = LaunchConfiguration('use_composition')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # use_namespace = LaunchConfiguration('use_namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        # 'use_sim_time': test_time,
        'yaml_filename': map_yaml_file}
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    bringup_cmd_node = Node(
        condition=IfCondition(use_composition),
        package='nav2_bringup',
        executable='composed_bringup',
        output='screen',
        parameters=[configured_params, {'autostart': autostart}],
        prefix=['xterm -e gdb -ex run --args'],
        remappings=remappings),
    ld = LaunchDescription()
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(bringup_cmd_node)
    return ld
