"""Orin Humble perception graph. Not the ThinkPad hardware_pick launch.

D555 (librealsense DDS) -> transport adapter (mapped raw, no JPEG/PNG) ->
Orin preprocessor B -> YOLO-World segmenter -> point filter -> detector.
Mid-360 via mid360.launch.

Planning, CPS, and scene TF stay on the laptop. Do not start the
ThinkPad hardware_pick graph here: that uses the x86_64 D555 wrapper.
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DDS_LIB = "/home/hku_reconova/librealsense_dds/lib"


def _truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _repo_root():
    env = os.environ.get("ROBOTICARM_ROOT", "").strip()
    if env and os.path.isdir(os.path.join(env, "deployment_ws", "config")):
        return os.path.abspath(env)
    probe = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cfg = os.path.join(probe, "deployment_ws", "config")
        src = os.path.join(probe, "src", "luggage_perception")
        if os.path.isdir(cfg) and os.path.isdir(src):
            return probe
        probe = os.path.dirname(probe)
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")
    )


def _prefixed_remaps(camera_name, pairs):
    result = []
    for source, target in pairs:
        result.append((source, target))
        result.append((camera_name + "/" + source, camera_name + "/" + target))
    return result


def _ld_library_path():
    parts = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
             if p and p != DDS_LIB]
    return os.pathsep.join([DDS_LIB] + parts)


def _d555_nodes(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration("start_d555").perform(context)):
        return []
    namespace = LaunchConfiguration("camera_namespace").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    serial = LaunchConfiguration("serial_no").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    cfg = LaunchConfiguration("d555_config_file").perform(context)
    if not cfg or not os.path.isfile(cfg):
        raise RuntimeError("D555 yaml not found: %s" % cfg)
    with open(cfg, "r", encoding="utf-8") as handle:
        params = yaml.safe_load(handle) or {}
    params.update({
        "camera_name": camera_name,
        "serial_no": serial,
        "use_sim_time": False,
        "rgb_camera.color_profile": profile,
        "depth_module.depth_profile": profile,
        "depth_module.infra_profile": profile,
        "pointcloud.enable": False,
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_color1": False,
        "publish_tf": True,
        "tf_publish_rate": 0.0,
        # D555 DDS keeps the sensor open after a failed start. Reset
        # before the wrapper opens 640x360 so open-streams is not rejected
        # with "Sensor is streaming".
        "initial_reset": True,
        "image_transport.publisher.enable_pub_plugins": ["image_transport/raw"],
    })
    remaps = _prefixed_remaps(camera_name, [
        ("color/image_raw", "color/image_hw"),
        ("color/image_raw/compressed", "color/image_hw/compressed"),
        ("color/camera_info", "color/camera_info_hw"),
        ("aligned_depth_to_color/image_raw",
         "aligned_depth_to_color/image_hw"),
        ("aligned_depth_to_color/image_raw/compressed",
         "aligned_depth_to_color/image_hw/compressed"),
        ("aligned_depth_to_color/camera_info",
         "aligned_depth_to_color/camera_info_hw"),
    ])
    ld_path = _ld_library_path()
    return [
        LogInfo(msg=(
            "[orin_perception] D555 DDS lib=%s config=%s serial=%s profile=%s"
            % (DDS_LIB, cfg, serial, profile)
        )),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=namespace,
            name=camera_name,
            output="screen",
            emulate_tty=True,
            remappings=remaps,
            additional_env={"LD_LIBRARY_PATH": ld_path},
            prefix="env LD_LIBRARY_PATH=" + ld_path,
            parameters=[params],
        ),
        Node(
            package="luggage_perception",
            executable="d555_transport_adapter_node.py",
            name="d555_transport_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "camera_namespace": namespace,
                "camera_name": camera_name,
                "jpeg_quality": int(
                    LaunchConfiguration("jpeg_quality").perform(context)),
                "publish_raw": True,
                "publish_compressed": False,
                "require_clock_lock": True,
            }],
        ),
    ]


def generate_launch_description():
    repo = _repo_root()
    perc_share = get_package_share_directory("luggage_perception")
    desc_share = get_package_share_directory("luggage_description")
    default_scene = os.path.join(desc_share, "config", "scene_tf.yaml")
    if not os.path.isfile(default_scene):
        default_scene = os.path.join(
            desc_share, "config", "scene_tf.yaml.example")
    orin_pp = os.path.join(perc_share, "config", "preprocessor_d555_orin.yaml")
    site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")
    base_pp = os.path.join(perc_share, "config", "sensor_preprocessor.yaml")
    semantic = os.path.join(perc_share, "config", "semantic_segmenter.yaml")
    d555_yaml = os.path.join(repo, "deployment_ws", "config", "d555_orin.yaml")
    mid360_json = os.path.join(
        repo, "deployment_ws", "config", "MID360s_config.json")
    mid360_launch = os.path.join(desc_share, "launch", "mid360.launch.py")
    start_perception = IfCondition(LaunchConfiguration("start_perception"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_tf_config", default_value=default_scene),
            DeclareLaunchArgument("pick_runtime_config", default_value=os.path.join(
                desc_share, "config", "pick_runtime.site.yaml")),
            DeclareLaunchArgument(
                "preprocessor_config",
                default_value=orin_pp,
                description=(
                    "Orin default is mapped-raw site B "
                    "(preprocessor_d555_orin.yaml). Compressed site B is %s. "
                    "Do not pass preprocessor_d555_replay.yaml (use_sim_time true)."
                    % site_pp
                ),
            ),
            DeclareLaunchArgument(
                "d555_config_file", default_value=d555_yaml),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=mid360_json,
                description="Site Mid-360 JSON (host 192.168.1.5).",
            ),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="d555"),
            DeclareLaunchArgument("serial_no", default_value="419222302385"),
            DeclareLaunchArgument("profile", default_value="640,360,15"),
            DeclareLaunchArgument("jpeg_quality", default_value="80"),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument(
                "semantic_device",
                default_value="cuda",
                description=(
                    "YOLO-World torch device. Orin live pick is CUDA-only; "
                    "CPU is ~1.2 s/frame and is rejected by orin_perception.sh."
                ),
            ),
            DeclareLaunchArgument(
                "publish_overlay",
                default_value="true",
                description=(
                    "Draw YOLO boxes on /luggage/semantic/overlay for debug. "
                    "Pass publish_overlay:=false to skip the extra image."
                ),
            ),
            LogInfo(msg=(
                "[orin_perception] Humble D555+YOLO+Livox. No CPS, no "
                "move_group, no scene_hardware. ROS_DOMAIN_ID=7."
            )),
            OpaqueFunction(function=_d555_nodes),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mid360_launch),
                launch_arguments={
                    "user_config_path": LaunchConfiguration(
                        "user_config_path"),
                    "frame_id": "livox_frame",
                    "xfer_format": "0",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_livox")),
            ),
            Node(
                package="luggage_perception",
                executable="sensor_preprocessor_node.py",
                name="sensor_preprocessor",
                output="screen",
                parameters=[
                    base_pp,
                    LaunchConfiguration("preprocessor_config"),
                    {"use_sim_time": False},
                ],
                condition=start_perception,
            ),
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
                        "workspace_accept_enabled": False,
                        "publish_overlay": ParameterValue(
                            LaunchConfiguration("publish_overlay"),
                            value_type=bool,
                        ),
                        "max_rate_hz": 0.0,
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
                    "cloud_max_age_sec": 8.0,
                    "estimate_retry_count": 4,
                    "estimate_retry_period_sec": 0.25,
                    "suitcase_update_timeout_sec": 0.0,
                    "crop_to_workspace": False,
                    "top_surface_mode": "dynamic",
                }, LaunchConfiguration("pick_runtime_config")],
                condition=start_perception,
            ),
        ]
    )
