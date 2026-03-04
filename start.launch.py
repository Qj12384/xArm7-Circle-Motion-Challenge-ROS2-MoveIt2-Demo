import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml(package_name, *paths):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, *paths)
    try:
        with open(abs_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return {}

def generate_launch_description():
    xarm_desc_path = get_package_share_directory('xarm_description')
    xarm_moveit_path = get_package_share_directory('xarm_moveit_config')
    
    # 1. Process URDF from xacro
    urdf_xacro = os.path.join(xarm_desc_path, 'urdf', 'xarm_device.urdf.xacro')
    urdf_doc = xacro.process_file(urdf_xacro, mappings={'dof': '7', 'robot_type': 'xarm'})
    robot_description = {'robot_description': urdf_doc.toxml()}
    
    # 2. Process SRDF and Inject the 'circle' Goal State
    srdf_xacro = os.path.join(xarm_moveit_path, 'srdf', 'xarm.srdf.xacro')
    srdf_doc = xacro.process_file(srdf_xacro, mappings={'dof': '7', 'robot_type': 'xarm'})
    srdf_xml = srdf_doc.toxml()
    
    circle_state = """
    <group_state name="circle" group="xarm7">
        <joint name="joint1" value="0.0"/>
        <joint name="joint2" value="-0.4"/>
        <joint name="joint3" value="0.0"/>
        <joint name="joint4" value="0.7"/>
        <joint name="joint5" value="0.0"/>
        <joint name="joint6" value="1.1"/>
        <joint name="joint7" value="0.0"/>
    </group_state>
    """
    patched_srdf = srdf_xml.replace('</robot>', circle_state + '</robot>')
    robot_description_semantic = {'robot_description_semantic': patched_srdf}
    
    # 3. Load configurations explicitly from the xarm7 subdirectory
    kinematics = {'robot_description_kinematics': load_yaml('xarm_moveit_config', 'config', 'xarm7', 'kinematics.yaml')}
    joint_limits = {'robot_description_planning': load_yaml('xarm_moveit_config', 'config', 'xarm7', 'joint_limits.yaml')}
    
    ompl_planning = load_yaml('xarm_moveit_config', 'config', 'xarm7', 'ompl_planning.yaml')
    if ompl_planning:
        ompl_planning.update({'planning_plugin': 'ompl_interface/OMPLPlanner'})
    planning_pipelines = {'ompl': ompl_planning}
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    fake_controller_manager = {'moveit_controller_manager': 'moveit_fake_controller_manager/MoveItFakeControllerManager'}
    fake_controllers = load_yaml('xarm_moveit_config', 'config', 'xarm7', 'fake_controllers.yaml')

    # 4. Initialize Nodes
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            planning_pipelines,
            trajectory_execution,
            fake_controller_manager,
            fake_controllers
        ],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(xarm_moveit_path, 'rviz', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
        ]
    )
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['/fake_controller_joint_states', '/move_group/fake_controller_joint_states']}]
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'link_base']
    )
    
    circle_node = Node(
        package='avatar_challenge',
        executable='circle_node.py',
        name='circle_motion_controller',
        output='screen'
    )
    
    return LaunchDescription([
        rsp_node,
        jsp_node,
        static_tf,
        move_group_node,
        rviz_node,
        circle_node
    ])
