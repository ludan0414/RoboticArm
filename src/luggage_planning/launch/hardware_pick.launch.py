"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Real-cell pick graph: D555 + CPS executor + detect + MoveIt + vacuum.
No Gazebo. Person on e-stop. One CPS client (jazzy_real).

  ros2 launch luggage_planning hardware_pick.launch.py
  # or: deployment_ws/scripts/hardware_pick.sh
  # no GPU: script injects semantic_device:=cpu

Then in another terminal:

  ros2 run luggage_planning hardware_pick_driver.py --detect-only
  ros2 run luggage_planning hardware_pick_driver.py
  ros2 run luggage_planning hardware_pick_driver.py --observe-pose current
      # default: stay at live joints (no sim pickup_observe)
"""

from __future__ import annotations

import os

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_text(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


_RAW_COLOR = "/camera/d555/color/image_raw"
_RAW_DEPTH = "/camera/d555/aligned_depth_to_color/image_raw"


def _preprocessor_node(context, *args, **kwargs):
    """Subscribe raw images unless use_compressed:=true."""
    if not _bool_text(LaunchConfiguration("start_perception").perform(context)):
        return []
    use_compressed = _bool_text(
        LaunchConfiguration("use_compressed").perform(context))
    perc_share = get_package_share_directory("luggage_perception")
    if use_compressed:
        color = _RAW_COLOR + "/compressed"
        depth = _RAW_DEPTH + "/compressed"
    else:
        color = _RAW_COLOR
        depth = _RAW_DEPTH
    return [
        Node(
            package="luggage_perception",
            executable="sensor_preprocessor_node.py",
            name="sensor_preprocessor",
            output="screen",
            parameters=[
                os.path.join(perc_share, "config", "sensor_preprocessor.yaml"),
                LaunchConfiguration("preprocessor_config"),
                {
                    "use_sim_time": False,
                    "input.use_compressed": use_compressed,
                    "input.color_image": color,
                    "input.depth_image": depth,
                },
            ],
        )
    ]


def _semantic_srdf():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "S20_with_camera.srdf",
    )
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    xml = xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    return {"robot_description_semantic": xml}


def _robot_description(scene_tf_config):
    helper = os.path.join(
        get_package_prefix("luggage_description"),
        "lib",
        "luggage_description",
        "xacro_robot_with_scene_base",
    )
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf",
        "elfin_s20_with_camera.urdf.xacro",
    )
    return {
        "robot_description": ParameterValue(
            Command([helper, " ", xacro_path, " ", scene_tf_config]),
            value_type=str,
        )
    }


def _move_group(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_planning").perform(context)):
        return []
    if not _bool_text(LaunchConfiguration("use_moveit").perform(context)):
        return []
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ImportError:
        return [
            LogInfo(msg=(
                "[hardware_pick] MoveIt is not installed. Skipping move_group. "
                "Detect-only still works. For plan/pick: sudo apt install "
                "ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils "
                "ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl "
                "ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager"
            ))
        ]
    scene = LaunchConfiguration("scene_tf_config").perform(context)
    robot_description = _robot_description(scene)
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description
    params = dict(moveit_config.to_dict())
    # Humble sim yaml uses scalar plugin keys nested under ompl.
    # Jazzy move_group rejects those; overlay the site string-array file.
    jazzy_ompl = os.path.join(
        get_package_share_directory("elfin_moveit_config"),
        "config",
        "ompl_planning.jazzy.yaml",
    )
    with open(jazzy_ompl, "r", encoding="utf-8") as handle:
        overlay = yaml.safe_load(handle) or {}
    params.pop("planning_plugin", None)
    params.pop("request_adapters", None)
    ompl = dict(params.get("ompl") or {})
    ompl.pop("planning_plugin", None)
    ompl.pop("request_adapters", None)
    ompl.update(overlay)
    params["ompl"] = ompl
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                params,
                _semantic_srdf(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "use_sim_time": False,
                },
            ],
        )
    ]


def generate_launch_description():
    desc_share = get_package_share_directory("luggage_description")
    perc_share = get_package_share_directory("luggage_perception")
    exec_share = get_package_share_directory("elfin_trajectory_executor")
    default_scene = os.path.join(desc_share, "config", "scene_tf.yaml")
    if not os.path.isfile(default_scene):
        default_scene = os.path.join(desc_share, "config", "scene_tf.yaml.example")
    poses = os.path.join(desc_share, "config", "robot_poses.site.yaml")
    sim_poses = os.path.join(desc_share, "config", "robot_poses.yaml.example")
    live_pp = os.path.join(perc_share, "config", "preprocessor_d555_live.yaml")
    semantic = os.path.join(perc_share, "config", "semantic_segmenter.yaml")
    site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")
    start_perception = IfCondition(LaunchConfiguration("start_perception"))
    start_planning = IfCondition(LaunchConfiguration("start_planning"))
    start_d555 = IfCondition(LaunchConfiguration("start_d555"))
    start_executor = IfCondition(LaunchConfiguration("start_executor"))
    start_scene = IfCondition(LaunchConfiguration("start_scene"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_tf_config", default_value=default_scene),
            DeclareLaunchArgument("pick_runtime_config", default_value=os.path.join(
                desc_share, "config", "pick_runtime.site.yaml")),
            DeclareLaunchArgument(
                "robot_poses_config",
                default_value=poses,
                description=(
                    "Named joint poses for GoToRobotPose. Default is "
                    "robot_poses.site.yaml (observe_pose=current). "
                    "Do not pass %s on this cell; those pickup_observe "
                    "joints are simulation." % sim_poses
                ),
            ),
            DeclareLaunchArgument(
                "observe_pose_name",
                default_value="current",
                description=(
                    "Default GoToRobotPose name when the goal is empty. "
                    "current/here = stay. pickup_observe requires measured "
                    "values in robot_poses_config."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument(
                "start_perception",
                default_value="true",
                description=(
                    "Start the perception role: preprocessor, semantic "
                    "segmenter, semantic point filter, and luggage detector."
                ),
            ),
            DeclareLaunchArgument(
                "start_planning",
                default_value="true",
                description=(
                    "Start the planning role: scene manager, waypoint "
                    "generator, motion planner, vacuum controller, and "
                    "MoveIt when use_moveit is true."
                ),
            ),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("start_executor", default_value="true"),
            DeclareLaunchArgument(
                "start_scene",
                default_value="true",
                description=(
                    "scene_hardware (RSP + container TF). Set false when "
                    "record_site already publishes the same tree."
                ),
            ),
            DeclareLaunchArgument(
                "preprocessor_config",
                default_value=site_pp,
                description=(
                    "实机 preprocessor. Default B is "
                    "preprocessor_d555_site.yaml (motion_gate off). "
                    "Profile A (motion_gate on) is preprocessor_config:=%s. "
                    "Do not use preprocessor_d555_replay.yaml for live pick "
                    "(use_sim_time true)." % live_pp
                ),
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="false",
                description=(
                    "false: preprocessor subscribes mapped raw "
                    "/camera/d555/.../image_raw. true: JPEG/PNG "
                    "image_raw/compressed (old bags and recorders only)."
                ),
            ),
            DeclareLaunchArgument(
                "publish_overlay",
                default_value="true",
                description=(
                    "Publish /luggage/semantic/overlay. semantic_segmenter.yaml "
                    "sets this false for sim-eval CPU hygiene; site detect "
                    "needs the image. Pass publish_overlay:=false to drop it."
                ),
            ),
            DeclareLaunchArgument("semantic_device", default_value="cuda"),
            DeclareLaunchArgument(
                "max_rate_hz",
                default_value="0.0",
                description=(
                    "YOLO cap. 0 follows the camera (~15 Hz). "
                    "A positive value drops frames closer than 1/max_rate_hz."
                ),
            ),
            DeclareLaunchArgument(
                "execution_backend",
                default_value="servo_j",
                description=(
                    "Real executor backend. ServoJ streams the timed trajectory; "
                    "waypoint remains an explicit compatibility override."
                ),
            ),
            DeclareLaunchArgument(
                "servo_j_servo_time",
                default_value="0.02",
                description="ServoJ fixed-grid period (s). Used only when backend=servo_j.",
            ),
            DeclareLaunchArgument(
                "servo_j_lookahead_time",
                default_value="0.1",
                description="ServoJ lookahead (s). Used only when backend=servo_j.",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.10"),
            DeclareLaunchArgument("robot_port", default_value="10003"),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="D555 RGB profile. Do not use 896x504@30.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="D555 depth profile. Match colour.",
            ),
            LogInfo(
                msg=(
                    "[hardware_pick] No Gazebo. e-stop person required. "
                    "CPS via jazzy_real only. D555 colour+aligned depth. "
                    "Ctrl+C does not BlackOut. Observe pose default=current "
                    "(no sim pickup_observe swing)."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "jazzy_real.launch.py")
                ),
                launch_arguments={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robot_port": LaunchConfiguration("robot_port"),
                    # Huayan WayPoint min ~1 deg/s. 10/20 made MoveIt TOTG
                    # crawl and still hit 20070; site Gate 5 uses 30/60.
                    "default_velocity_deg": "30.0",
                    "max_velocity_deg": "60.0",
                    "command_acceleration_deg": "60.0",
                    "execution_backend": LaunchConfiguration("execution_backend"),
                    "servo_j_servo_time": LaunchConfiguration("servo_j_servo_time"),
                    "servo_j_lookahead_time": LaunchConfiguration(
                        "servo_j_lookahead_time"
                    ),
                }.items(),
                condition=start_executor,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "scene_hardware.launch.py")
                ),
                launch_arguments={
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_sim_time": "false",
                }.items(),
                condition=start_scene,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "d555_rgbd.launch.py")
                ),
                launch_arguments={
                    "color_profile": LaunchConfiguration("color_profile"),
                    "depth_profile": LaunchConfiguration("depth_profile"),
                }.items(),
                condition=start_d555,
            ),
            OpaqueFunction(function=_preprocessor_node),
            Node(
                package="luggage_perception",
                executable="semantic_segmenter_node.py",
                name="semantic_segmenter",
                output="screen",
                parameters=[
                    semantic,
                    {
                        "use_sim_time": False,
                        "device": LaunchConfiguration("semantic_device"),
                        "self_body_camera_frame": "d555_color_optical_frame",
                        "require_backend": "yolo_world",
                        # yaml default true is a sim FP gate against the
                        # scene_tf pickup square. Site crop is YOLO bbox
                        # → depth, not that square.
                        "workspace_accept_enabled": False,
                        # After yaml: yaml publish_overlay is false (eval).
                        "publish_overlay": ParameterValue(
                            LaunchConfiguration("publish_overlay"),
                            value_type=bool,
                        ),
                        # 0 follows the 15 Hz camera. A 2 Hz cap drops frames
                        # even when inference is ~64 ms.
                        "max_rate_hz": ParameterValue(
                            LaunchConfiguration("max_rate_hz"),
                            value_type=float,
                        ),
                    },
                ],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="semantic_point_filter_node.py",
                name="semantic_point_filter",
                output="screen",
                parameters=[semantic, {"use_sim_time": False}],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="luggage_detector_node.py",
                name="luggage_detector",
                output="screen",
                parameters=[{
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_semantic": True,
                    "use_sim_time": False,
                    "depth_topic": "/luggage/preprocessed/camera/depth/image",
                    # CPU YOLO + D555 host-stamp lag: 8 s rejected still
                    # frames (DETECT_STALE_CLOUD). Site pick waits longer.
                    "cloud_max_age_sec": 30.0,
                    "estimate_retry_count": 6,
                    "estimate_retry_period_sec": 2.0,
                    "suitcase_update_timeout_sec": 0.0,
                    "crop_to_workspace": False,
                    # DYNAMIC-SUCTION ST-1: hardware top surface comes from
                    # the YOLO instance ROI + aligned-depth component; the
                    # cargo-cloud RANSAC/PCA legacy path stays the default
                    # everywhere else. scene_tf remains TF/collision/safety
                    # authority only.
                    "top_surface_mode": "dynamic",
                }, LaunchConfiguration("pick_runtime_config")],
                condition=start_perception,
            ),
            Node(
                package="luggage_planning",
                executable="scene_manager_node.py",
                name="scene_manager",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "world_frame": "world",
                    "pickup_object_id": "pickup_box",
                    "auto_sync": True,
                }],
                condition=start_planning,
            ),
            Node(
                package="luggage_planning",
                executable="waypoint_generator_node.py",
                name="waypoint_generator",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "world_frame": "world",
                    "place_slot_frame": "elfin_base_link",
                    "use_perception_approach": False,
                }],
                condition=start_planning,
            ),
            Node(
                package="luggage_planning",
                executable="motion_planner_node.py",
                name="motion_planner",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "robot_poses_config": LaunchConfiguration("robot_poses_config"),
                    "default_observe_pose": LaunchConfiguration("observe_pose_name"),
                    "named_pose_duration": 8.0,
                    "named_pose_max_vel": 1.0,
                    "velocity_scaling": 0.15,
                    "acceleration_scaling": 0.10,
                    # Extra excursion heuristics are advisory for this real
                    # cell. MoveIt limits/collisions and wrist checks remain.
                    "max_segment_joint_excursion_rad": 0.0,
                    "cartesian_excursion_rad_per_m": 0.0,
                    "fjt_action": "/elfin_arm_controller/follow_joint_trajectory",
                }],
                condition=start_planning,
            ),
            Node(
                package="luggage_planning",
                executable="vacuum_controller_node.py",
                name="vacuum_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "backend": "hardware",
                    "seal_timeout_sec": 8.0,
                    "follow_rate_hz": 10.0,
                }, LaunchConfiguration("pick_runtime_config")],
                condition=start_planning,
            ),
            OpaqueFunction(function=_move_group),
        ]
    )
