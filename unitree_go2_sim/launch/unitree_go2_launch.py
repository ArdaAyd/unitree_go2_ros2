import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_min_normal_force = DeclareLaunchArgument(
        "min_normal_force", default_value="0.0",
        description="Temasin BASLAMASI icin gereken kuvvet (N). 0 = esik kapali.")
    declare_foot_friction = DeclareLaunchArgument(
        "foot_friction", default_value="0.6",
        description="Ayak-zemin surtunme katsayisi; dusuk deger kaygan zemin demek")
    declare_contact_source_topic = DeclareLaunchArgument(
        "contact_source_topic", default_value="foot_contacts",
        description="Durum tahmincisinin dinleyecegi temas topigi: "
                    "'foot_contacts' (Gazebo sensorleri) veya "
                    "'foot_contacts/gait' (yuruyus fazi)")
    declare_contact_release_force = DeclareLaunchArgument(
        "contact_release_force", default_value="5.0",
        description="Temasin BITMESI icin altina inilmesi gereken kuvvet (N); "
                    "histerezis, esik etrafindaki zipla(chatter)mayi onler.")
    declare_contact_timeout = DeclareLaunchArgument(
        "contact_timeout", default_value="0.05",
        description="How long a foot stays 'in contact' after the last contact message (s)")
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )
    declare_command_interface = DeclareLaunchArgument(
        "command_interface",
        default_value="effort",
        description="ros2_control joint command interface: effort or position",
    )
    declare_disable_camera = DeclareLaunchArgument(
        "disable_camera", default_value="false", description="Leave out the mono camera"
    )
    declare_disable_d455 = DeclareLaunchArgument(
        "disable_d455", default_value="false", description="Leave out the RealSense D455"
    )
    declare_disable_lidar_l1 = DeclareLaunchArgument(
        "disable_lidar_l1", default_value="false", description="Leave out the 4D Lidar L1"
    )
    declare_disable_velodyne_lidar = DeclareLaunchArgument(
        "disable_velodyne_lidar", default_value="false", description="Leave out the Velodyne lidar"
    )

    # Description nodes and parameters
    description_command = Command([
        "xacro ", LaunchConfiguration("unitree_go2_description_path"),
        " robot_controllers:=", LaunchConfiguration("ros_control_file"),
        " command_interface:=", LaunchConfiguration("command_interface"),
        " FOOT_FRICTION:=", LaunchConfiguration("foot_friction"),
        " disable_camera:=", LaunchConfiguration("disable_camera"),
        " disable_d455:=", LaunchConfiguration("disable_d455"),
        " disable_lidar_l1:=", LaunchConfiguration("disable_lidar_l1"),
        " disable_velodyne_lidar:=", LaunchConfiguration("disable_velodyne_lidar"),
    ])
    robot_description = {"robot_description": description_command}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[ robot_description ],
    )
    
    # CHAMP controller nodes
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": True},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": description_command},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # TEMAS YAPILANDIRMASI — Asama 1 madde 1.2'de olculup tek bir secime
    # baglanacak. Su anki ETKIN durum:
    #   min_normal_force = 0.0  (launch, dugumun 15.0 varsayilanini EZIYOR)
    #   contact_release_force = 5.0
    # min > release olmadigi icin histerezis KAPALI; dugum bu durumda uyari
    # basiyor. Yani fiilen "sensor, esik kapali" yapilandirmasi kullaniliyor.
    #
    # PILOT GOZLEM (DOGRULANMADI): esik kapaliyken ayak basina temas orani
    # %63-72 olculmustu (tiris yuruyusunde nominal ~%50 beklenir); salinimdaki
    # ayagin siyirma temaslarinin da sayilmasi muhtemel sebep. Bunun bacak
    # odometrisine etkisi OLCULMEDI. Uc secenek (A: esik kapali, B: histerezis
    # 15/5 N, C: yuruyus fazi) madde 1.2'de karsilastirilacak.
    foot_contacts_bridge_node = Node(
        package="unitree_application",
        executable="foot_contacts_bridge",
        output="screen",
        parameters=[
            {"min_normal_force": LaunchConfiguration("min_normal_force")},
            {"contact_release_force": LaunchConfiguration("contact_release_force")},
            {"contact_timeout": LaunchConfiguration("contact_timeout")},
        ],
    )

    # Durum tahmincisinin hangi temas kaynagini kullanacagi:
    #   sensor -> Gazebo temas sensorleri (/foot_contacts)
    #   gait   -> yuruyus fazindan uretilen tahmin (/foot_contacts/gait)
    # Gazebo sensoru salinimdaki ayagin siyirma temaslarini da bildirdigi icin
    # ayak basina temas orani %63-72 cikiyor (tirista ~%50 olmali) ve bacak
    # odometrisi bozuluyor; olcumler icin bkz. DURUM.md.
    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        remappings=[("foot_contacts", LaunchConfiguration("contact_source_topic"))],
        parameters=[
            {"orientation_from_imu": True},
            {"urdf": description_command},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # Go2 static frame connection (map -> odom)
    # SLAM dugumleri (RTAB-Map, FAST-LIO2 sarmallari) map->odom'u KENDILERI
    # yayinlar. Bu statik birim donusum acik kalirsa ayni donusumun iki
    # yayincisi olur ve TF agaci tutarsizlasir. SLAM kosulurken
    # publish_map_odom:=false verilmeli.
    declare_publish_map_odom = DeclareLaunchArgument(
        "publish_map_odom", default_value="true",
        description="Statik map->odom yayinla. SLAM kosarken false yapin.")
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        condition=IfCondition(LaunchConfiguration('publish_map_odom')),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # 'gui' argumani tanimliydi ama gz_args'a hic baglanmamisti: gui:=false
            # verilse bile Gazebo penceresi aciliyordu. PythonExpression ile
            # gui:=false oldugunda '-s' (yalnizca sunucu) ekleniyor.
            'gz_args': [LaunchConfiguration('world'), ' -r',
                        PythonExpression(["'' if '", LaunchConfiguration('gui'),
                                          "'.lower() in ('true','1') else ' -s'"])]
        }.items(),
    )
    
    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )
    
    # Bridge ROS 2 and Gazebo Sim
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/odom/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/lf_foot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/rf_foot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/lh_foot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/rh_foot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
    )

    # Optional sensor bridges, gated by the same flags the description uses
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[ '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image', ],
        condition=UnlessCondition(LaunchConfiguration('disable_camera')),
    )

    d455_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='d455_bridge',
        output='screen',
        arguments=[
            '/d455/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        condition=UnlessCondition(LaunchConfiguration('disable_d455')),
    )

    lidar_l1_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_l1_bridge',
        output='screen',
        arguments=[ '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked', ],
        condition=UnlessCondition(LaunchConfiguration('disable_lidar_l1')),
    )

    velodyne_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='velodyne_bridge',
        output='screen',
        arguments=[ '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked', ],
        condition=UnlessCondition(LaunchConfiguration('disable_velodyne_lidar')),
    )

    # Use spawner nodes directly to handle the configuration step. (load → configure → activate)
    controller_spawner_js = TimerAction(
        period=20.0,  # Wait for Gazebo to fully initialize
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",  # Longer timeout
                    "joint_states_controller",  # No --inactive flag to ensure full activation
                ],
            )
        ]
    )

    controller_spawner_effort = TimerAction(
        period=30.0,  # Wait 5 seconds after joint_states_controller
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",  # Longer timeout
                    "joint_group_effort_controller",  # No --inactive flag to ensure full activation
                ],
            )
        ]
    )
    
    # Shell script to manually check controller status 
    controller_status_check = TimerAction(
        period=25.0,  # Check status after controllers should be loaded
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Checking controller status:' && ros2 control list_controllers"],
                output='screen',
            )
        ]
    )
    
    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_min_normal_force,
            declare_foot_friction,
            declare_contact_source_topic,
            declare_contact_release_force,
            declare_contact_timeout,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path, 
            declare_command_interface,
            declare_disable_camera,
            declare_disable_d455,
            declare_disable_lidar_l1,
            declare_disable_velodyne_lidar,

            # Parameters
            SetParameter(name="use_sim_time", value=use_sim_time),

            # Gazebo and robot nodes first
            gz_sim,
            robot_state_publisher_node,
            gazebo_spawn_robot,
            gazebo_bridge,
            camera_bridge,
            d455_bridge,
            lidar_l1_bridge,
            velodyne_bridge,
            
            # CHAMP controller nodes
            quadruped_controller_node,
            foot_contacts_bridge_node,
            state_estimator_node,
            
            # EKF nodes for localization
            base_to_footprint_ekf,
            footprint_to_odom_ekf,
            
            # Static TF publisher for frame connections
            declare_publish_map_odom,
            map_to_odom_tf_node,
            
            # Controller spawners that handle the complete lifecycle
            controller_spawner_js,
            controller_spawner_effort,
            controller_status_check,
            
            # Visualization (only if rviz flag is set)
            rviz2,
        ]
    )
