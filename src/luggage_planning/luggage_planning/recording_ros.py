"""ROS node-layer subscription wiring for passive commissioning recording."""
import json
import os
from pathlib import Path
import subprocess
import zipfile

from rcl_interfaces.msg import Log, ParameterEvent
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from tf2_msgs.msg import TFMessage
from luggage_msgs.msg import VacuumState

from .telemetry_journal import TelemetryJournal


def attach_recorder(node, journal, session):
    recorder = TelemetryJournal(journal, session)
    reported = []

    def receive(topic, message):
        payload = message_to_ordereddict(message)
        if isinstance(message, String):
            try:
                payload = json.loads(message.data)
            except (ValueError, TypeError):
                pass
        if not recorder.write(topic, payload) and recorder.error and not reported:
            reported.append(True)
            node.get_logger().error('TELEMETRY_RECORDING_FAILED: '+recorder.error)

    topics = {'/joint_states': JointState, '/vacuum/state': VacuumState,
        '/vacuum/io': String, '/vacuum/di0': Bool, '/vacuum/do0': Bool, '/vacuum/do1': Bool,
        '/elfin/cps_telemetry': String, '/elfin/hardware_status': String,
        '/trajectory_executor/events': String, '/trajectory_executor/status': String,
        '/trajectory_executor/command': String, '/motion_planner/last_boundary': String,
        '/tf': TFMessage, '/tf_static': TFMessage, '/rosout': Log,
        '/parameter_events': ParameterEvent}
    latched = {'/tf_static', '/elfin/hardware_status', '/trajectory_executor/events',
               '/trajectory_executor/status', '/motion_planner/last_boundary'}
    recorder.subscriptions = []
    for topic, kind in topics.items():
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT if topic == '/tf'
                        else ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL if topic in latched
                        else DurabilityPolicy.VOLATILE)
        recorder.subscriptions.append(node.create_subscription(kind, topic,
            lambda msg, topic=topic: receive(topic, msg), qos))
    # Preserve the actual dirty runtime sources, not just a HEAD that may omit
    # locally deployed fixes. Never include weights, recordings, or credentials.
    try:
        root = subprocess.check_output(['git', 'rev-parse', '--show-toplevel'],
            cwd=Path(__file__).resolve().parent, text=True, timeout=3).strip()
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True, timeout=3).strip()
        bundle = recorder.path.with_suffix('.sources.zip')
        with zipfile.ZipFile(bundle, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
            for prefix in ('src/luggage_planning/luggage_planning', 'src/luggage_planning/scripts',
                           'deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor'):
                for source in (Path(root)/prefix).glob('*.py'):
                    archive.write(source, str(source.relative_to(root)))
            for name in ('src/luggage_planning/launch/hardware_pick.launch.py',
                         'src/elfin_moveit_config/config/joint_limits.yaml',
                         'deployment_ws/config/scene_tf.loading_site.yaml'):
                if (Path(root)/name).is_file():
                    archive.write(Path(root)/name, name)
        recorder.write('recorder', dict(event='environment', revision=revision,
            sources=str(bundle), ros_distro=os.environ.get('ROS_DISTRO'),
            ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), expected_topics=list(topics)))
    except (OSError, subprocess.SubprocessError) as exc:
        recorder.write('recorder', dict(event='source_capture_failed', error=str(exc)))
    node.get_logger().info('Continuous telemetry: '+str(recorder.path))
    return recorder
