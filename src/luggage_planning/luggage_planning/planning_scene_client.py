#!/usr/bin/env python3
"""MoveIt PlanningScene client for pickup box, static scene, and ACM.

Imports moveit_msgs at the top level by design (same class as
ros_message_adapters / motion_executor); algorithm modules must not import
this. Message construction is separated into pure functions so the shapes
are unit-testable without a running move_group.

Three tiers used by the closed loop:
- ``add_pickup_box``    collision-only, before BuildMotionSequence so
                        pre_grasp/approach plan around the box
- ``attach_pickup_box`` box becomes an AttachedCollisionObject on the
                        suction link (ACM allows panel contact)
- ``detach_and_remove`` release + remove from the scene

Place support:
- ``add_collision_mesh`` / ``add_collision_box`` / ``remove_object``
- ``set_acm_pairs`` fetch-merge-replace of AllowedCollisionMatrix
"""

from __future__ import division

import threading
import math
import time

from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose as PoseMsg
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from .pose import Quaternion
from .suction_candidate_waypoints import quat_mul,quat_rotate

BOX_OBJECT_ID = "pickup_box"
DEFAULT_ATTACH_LINK = "suction_contact_frame"
# Only the suction hardware may touch the carried box. Allowing arm links 5/6
# hid exactly the payload-vs-arm contact that this transfer must reject.
DEFAULT_TOUCH_LINKS = (
    "suction_panel",
    "suction_contact_frame",
)
ARM_LINKS_NEVER_TOUCH_PAYLOAD = ("elfin_link5", "elfin_link6")


def build_box_primitive(box_size):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(box_size[0]), float(box_size[1]),
                            float(box_size[2])]
    return primitive


def _pose_msg(xyz, quat):
    pose = PoseMsg()
    pose.position.x, pose.position.y, pose.position.z = (
        float(xyz[0]), float(xyz[1]), float(xyz[2]))
    pose.orientation.x, pose.orientation.y = float(quat[0]), float(quat[1])
    pose.orientation.z, pose.orientation.w = float(quat[2]), float(quat[3])
    return pose


def build_add_scene(box_id, xyz, quat, box_size, frame_id="world"):
    """Collision-only PlanningScene diff adding the box."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [build_collision_object(
        box_id, xyz, quat, box_size, frame_id,
        CollisionObject.ADD)]
    return scene


def _xyz_quat_from_pose(pose):
    if pose is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    pos = pose.position
    ori = pose.orientation
    return (
        [float(pos.x), float(pos.y), float(pos.z)],
        [float(ori.x), float(ori.y), float(ori.z), float(ori.w)],
    )


def _is_origin_pose(xyz, quat):
    return (sum(abs(float(v)) for v in xyz) < 1e-9
            and abs(float(quat[0])) < 1e-9
            and abs(float(quat[1])) < 1e-9
            and abs(float(quat[2])) < 1e-9)


def summarize_collision_object(obj):
    """Compact collision-object record. Mesh vertices are omitted."""
    row = {
        "id": str(getattr(obj, "id", "") or ""),
        "frame": str(getattr(getattr(obj, "header", None), "frame_id", "") or ""),
        "type": "unknown",
    }
    primitives = list(getattr(obj, "primitives", None) or [])
    primitive_poses = list(getattr(obj, "primitive_poses", None) or [])
    obj_xyz, obj_quat = _xyz_quat_from_pose(getattr(obj, "pose", None))
    if primitives:
        prim = primitives[0]
        prim_xyz, prim_quat = _xyz_quat_from_pose(
            primitive_poses[0] if primitive_poses else None)
        if _is_origin_pose(prim_xyz, prim_quat) and not _is_origin_pose(
                obj_xyz, obj_quat):
            xyz, quat = obj_xyz, obj_quat
        else:
            xyz, quat = prim_xyz, prim_quat
        row.update({
            "type": "box" if int(getattr(prim, "type", 0) or 0) == int(
                SolidPrimitive.BOX) else int(getattr(prim, "type", 0) or 0),
            "size": [float(v) for v in (prim.dimensions or [])],
            "xyz": xyz,
            "quat": quat,
            "xyz_object": obj_xyz,
            "xyz_primitive": prim_xyz,
        })
        return row
    meshes = list(getattr(obj, "meshes", None) or [])
    mesh_poses = list(getattr(obj, "mesh_poses", None) or [])
    if meshes:
        pose = mesh_poses[0] if mesh_poses else getattr(obj, "pose", None)
        xyz, quat = _xyz_quat_from_pose(pose)
        mesh = meshes[0]
        row.update({
            "type": "mesh",
            "n_triangles": len(getattr(mesh, "triangles", None) or []),
            "xyz": xyz,
            "quat": quat,
        })
        return row
    xyz, quat = _xyz_quat_from_pose(getattr(obj, "pose", None))
    row.update({"xyz": xyz, "quat": quat})
    return row


def summarize_planning_scene(scene):
    """World/attached/robot-state summary for replay. No mesh vertices."""
    if scene is None:
        return {"error": "no planning scene"}
    world = getattr(scene, "world", None)
    objects = list(getattr(world, "collision_objects", None) or [])
    robot = getattr(scene, "robot_state", None)
    attached = list(getattr(robot, "attached_collision_objects", None) or [])
    joints = getattr(robot, "joint_state", None) if robot is not None else None
    names = list(getattr(joints, "name", None) or [])
    positions = [float(v) for v in (getattr(joints, "position", None) or [])]
    octomap = getattr(world, "octomap", None)
    octo_info = None
    if octomap is not None and getattr(octomap, "octomap", None) is not None:
        blob = octomap.octomap
        octo_info = {
            "frame": str(getattr(getattr(octomap, "header", None), "frame_id", "")
                         or ""),
            "id": str(getattr(blob, "id", "") or ""),
            "binary_bytes": len(getattr(blob, "data", None) or b""),
        }
    return {
        "world_objects": [summarize_collision_object(obj) for obj in objects],
        "world_object_ids": [str(obj.id) for obj in objects],
        "attached": [str(item.object.id) for item in attached if item.object],
        "attached_geometry": [dict(
            link=item.link_name, touch_links=list(item.touch_links),
            object=summarize_collision_object(item.object))
            for item in attached if item.object],
        "robot_joints": {"name": names, "position": positions},
        "octomap": octo_info,
    }


def build_collision_object(box_id, xyz, quat, box_size, frame_id,
                           operation):
    obj = CollisionObject()
    obj.id = box_id
    obj.header.frame_id = frame_id
    obj.operation = operation
    obj.primitives = [build_box_primitive(box_size)]
    obj.primitive_poses = [_pose_msg(xyz, quat)]
    return obj


def build_remove_object(box_id):
    obj = CollisionObject()
    obj.id = box_id
    obj.operation = CollisionObject.REMOVE
    return obj


def mesh_msg_from_arrays(vertices, faces):
    """Build shape_msgs/Mesh. ``faces`` must be integer triplets."""
    msg = Mesh()
    msg.vertices = [
        PointMsg(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        for v in vertices]
    msg.triangles = [
        MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
        for f in faces]
    return msg


def load_stl_mesh_msg(path, max_faces=0):
    """Load an STL into shape_msgs/Mesh. Optional quadric decimation."""
    import trimesh
    mesh = trimesh.load(path, force="mesh")
    if mesh is None:
        raise FileNotFoundError("trimesh failed to load %s" % path)
    if not hasattr(mesh, "faces"):
        raise ValueError("STL %s did not produce a triangular mesh" % path)
    n_faces = int(len(mesh.faces))
    limit = int(max_faces)
    if limit > 0 and n_faces > limit:
        try:
            mesh = mesh.simplify_quadric_decimation(limit)
        except TypeError:
            mesh = mesh.simplify_quadric_decimation(face_count=limit)
    return mesh_msg_from_arrays(mesh.vertices, mesh.faces)


def build_mesh_collision_object(obj_id, xyz, quat, mesh_msg,
                                frame_id="world",
                                operation=CollisionObject.ADD):
    obj = CollisionObject()
    obj.id = obj_id
    obj.header.frame_id = frame_id
    obj.operation = operation
    obj.meshes = [mesh_msg]
    obj.mesh_poses = [_pose_msg(xyz, quat)]
    return obj


def build_attach_scene(box_id, xyz, quat, box_size, link_name,
                       touch_links=DEFAULT_TOUCH_LINKS, frame_id="world"):
    """Attach the (already added) box to ``link_name`` with an ACM entry."""
    scene = PlanningScene()
    scene.is_diff = True
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.object = build_collision_object(
        box_id, xyz, quat, box_size, frame_id, CollisionObject.ADD)
    attached.touch_links = [str(t) for t in touch_links]
    scene.robot_state.attached_collision_objects = [attached]
    # touch_links is the official way to allow panel contact. Do not send
    # a replacement AllowedCollisionMatrix: MoveIt 2 treats a non-empty
    # ACM in a diff as a wholesale replace and wipes the SRDF whitelist.
    scene.robot_state.is_diff = True
    return scene


def build_detach_scene(box_id, link_name=DEFAULT_ATTACH_LINK):
    """Detach the box and remove it from the world in one diff."""
    scene = PlanningScene()
    scene.is_diff = True
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.object.id = box_id
    attached.object.operation = CollisionObject.REMOVE
    scene.robot_state.attached_collision_objects = [attached]
    removed = CollisionObject()
    removed.id = box_id
    removed.operation = CollisionObject.REMOVE
    scene.world.collision_objects = [removed]
    scene.robot_state.is_diff = True
    return scene


def acm_index(acm, name):
    """Return the row index for ``name``, appending a False row if missing."""
    if name in acm.entry_names:
        return list(acm.entry_names).index(name)
    idx = len(acm.entry_names)
    acm.entry_names.append(name)
    for entry in acm.entry_values:
        entry.enabled.append(False)
    acm.entry_values.append(
        AllowedCollisionEntry(enabled=[False] * (idx + 1)))
    return idx


def set_acm_pair(acm, name_a, name_b, allowed):
    """Set a symmetric ACM entry. Mutates ``acm``."""
    ia = acm_index(acm, name_a)
    ib = acm_index(acm, name_b)
    width = len(acm.entry_names)
    for entry in acm.entry_values:
        while len(entry.enabled) < width:
            entry.enabled.append(False)
    acm.entry_values[ia].enabled[ib] = bool(allowed)
    acm.entry_values[ib].enabled[ia] = bool(allowed)
    return acm


def acm_pair_allowed(acm, name_a, name_b):
    names = list(acm.entry_names)
    if name_a not in names or name_b not in names:
        return False
    ia = names.index(name_a)
    ib = names.index(name_b)
    try:
        return bool(acm.entry_values[ia].enabled[ib])
    except (IndexError, AttributeError):
        return False


def build_acm_diff(acm):
    """PlanningScene diff that replaces the ACM with ``acm``.

    Callers must fetch the live matrix first; a partial ACM wipes the SRDF
    whitelist (MoveIt 2 treats a non-empty ACM in a diff as replace).
    """
    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = acm
    return scene


class PlanningSceneClient(object):
    """Thin caller for /apply_planning_scene with the three tiers."""

    def __init__(self, node, service_name="/apply_planning_scene",
                 get_service_name="/get_planning_scene",
                 callback_group=None):
        self._node = node
        kwargs = {}
        if callback_group is not None:
            kwargs["callback_group"] = callback_group
        self._client = node.create_client(
            ApplyPlanningScene, service_name, **kwargs)
        self._get = node.create_client(
            GetPlanningScene, get_service_name, **kwargs)

    def wait_ready(self, timeout_sec=30.0):
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def _apply(self, scene, timeout=10.0):
        if not self._client.wait_for_service(timeout_sec=timeout):
            return False, "apply_planning_scene unavailable"
        request = ApplyPlanningScene.Request()
        request.scene = scene
        event = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return False, "apply timeout"
        response = future.result()
        if response is None or not response.success:
            return False, "apply rejected"
        return True, "applied"

    def get_scene(self, components, timeout=5.0):
        if not self._get.wait_for_service(timeout_sec=timeout):
            return None
        request = GetPlanningScene.Request()
        request.components.components = int(components)
        event = threading.Event()
        future = self._get.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        response = future.result()
        if response is None:
            return None
        return response.scene

    def fetch_acm(self, timeout=5.0):
        scene = self.get_scene(
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX, timeout)
        if scene is None:
            return None
        return scene.allowed_collision_matrix

    # ------------------------------------------------------------------
    # Three tiers

    def add_pickup_box(self, xyz, quat, box_size, frame_id="world",
                       box_id=BOX_OBJECT_ID):
        return self._apply(
            build_add_scene(box_id, xyz, quat, box_size, frame_id))

    def attach_pickup_box(self, model_name, xyz, quat, box_size,
                          link_name=DEFAULT_ATTACH_LINK, frame_id="world",
                          box_id=BOX_OBJECT_ID):
        del model_name  # object id is fixed; model name kept for logs
        return self._apply(
            build_attach_scene(box_id, xyz, quat, box_size, link_name,
                               frame_id=frame_id))

    def detach_and_remove(self, model_name=None, box_id=BOX_OBJECT_ID):
        del model_name
        return self._apply(build_detach_scene(box_id))

    def add_collision_box(self, obj_id, xyz, quat, box_size,
                          frame_id="world", timeout=10.0):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [build_collision_object(
            obj_id, xyz, quat, box_size, frame_id, CollisionObject.ADD)]
        return self._apply(scene, timeout=timeout)

    def add_collision_boxes(self, boxes, frame_id="world", timeout=10.0):
        """Add many boxes in one PlanningScene diff.

        ``boxes`` is an iterable of dicts with ``id``, ``xyz``, ``size``,
        optional ``quat``.
        """
        scene = PlanningScene()
        scene.is_diff = True
        objects = []
        for box in boxes:
            quat = box.get("quat") or (0.0, 0.0, 0.0, 1.0)
            objects.append(build_collision_object(
                str(box["id"]), box["xyz"], quat, box["size"], frame_id,
                CollisionObject.ADD))
        scene.world.collision_objects = objects
        return self._apply(scene, timeout=timeout)

    def remove_objects(self, obj_ids, timeout=10.0):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            build_remove_object(str(obj_id)) for obj_id in obj_ids]
        return self._apply(scene, timeout=timeout)

    def ensure_doorway_clearance(self, boxes, require_payload=False, payload_size=None, timeout=5.0):
        """Apply/read back doorway geometry and the carried payload.

        Per operator instruction this gate does not read, rewrite or validate
        the ACM. MoveIt retains its normal trajectory collision checking.
        """
        boxes=list(boxes)
        if len(boxes)!=4:
            return False,'DOORWAY_SCENE: four frame guards required'
        ok,message=self.add_collision_boxes(boxes,frame_id='world',timeout=timeout)
        if not ok:
            return False,'DOORWAY_SCENE: '+message
        scene=self.get_scene(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                             PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,timeout=timeout)
        if scene is None:
            return False,'DOORWAY_SCENE: readback unavailable'
        objects={obj.id:obj for obj in scene.world.collision_objects}
        for expected in boxes:
            obj=objects.get(expected['id'])
            if (obj is None or obj.header.frame_id.lstrip('/')!='world' or len(obj.primitives)!=1
                    or len(obj.primitive_poses)!=1 or obj.primitives[0].type!=SolidPrimitive.BOX):
                return False,'DOORWAY_SCENE: missing/invalid '+expected['id']
            dimensions=list(obj.primitives[0].dimensions)
            if (len(dimensions)!=3 or not all(math.isfinite(v) and v>0 for v in dimensions)
                    or max(abs(a-b) for a,b in zip(dimensions,expected['size']))>1e-6):
                return False,'DOORWAY_SCENE: size mismatch '+expected['id']
            origin,orientation=_xyz_quat_from_pose(getattr(obj,'pose',None))
            if max(abs(v) for v in orientation)<1e-12 and max(abs(v) for v in origin)<1e-12:
                orientation=[0.,0.,0.,1.]
            position,quat=_xyz_quat_from_pose(obj.primitive_poses[0])
            q=Quaternion(*orientation)
            offset=quat_rotate(q,position)
            position=[a+b for a,b in zip(origin,offset)]
            actual=quat_mul(q,Quaternion(*quat))
            values=[actual.x,actual.y,actual.z,actual.w]
            if (not all(math.isfinite(v) for v in position+values)
                    or abs(sum(v*v for v in values)-1)>1e-6
                    or max(abs(a-b) for a,b in zip(position,expected['xyz']))>1e-5
                    or abs(abs(sum(a*b for a,b in zip(values,expected['quat'])))-1)>1e-6):
                return False,'DOORWAY_SCENE: pose mismatch '+expected['id']
        if require_payload:
            attached=[a for a in scene.robot_state.attached_collision_objects if a.object.id==BOX_OBJECT_ID]
            if len(attached)!=1 or attached[0].link_name!=DEFAULT_ATTACH_LINK:
                return False,'DOORWAY_PAYLOAD: attached pickup box not confirmed'
            primitives=attached[0].object.primitives
            if (len(primitives)!=1 or primitives[0].type!=SolidPrimitive.BOX or payload_size is None
                    or len(primitives[0].dimensions)!=3 or len(payload_size)!=3
                    or not all(math.isfinite(v) and v>0 for v in [*primitives[0].dimensions,*payload_size])
                    or max(abs(a-b) for a,b in zip(primitives[0].dimensions,payload_size))>1e-6):
                return False,'DOORWAY_PAYLOAD: attached box dimensions mismatch'
        return True,'doorway guard geometry and payload attachment verified'

    def add_collision_mesh(self, obj_id, xyz, quat, mesh_msg,
                           frame_id="world", timeout=30.0):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [build_mesh_collision_object(
            obj_id, xyz, quat, mesh_msg, frame_id, CollisionObject.ADD)]
        return self._apply(scene, timeout=timeout)

    def remove_object(self, obj_id, timeout=10.0):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [build_remove_object(obj_id)]
        return self._apply(scene, timeout=timeout)

    def set_acm_pairs(self, pairs, allowed, verify=True, timeout=5.0):
        """Set each (a, b) ACM pair. ``pairs`` is an iterable of name tuples."""
        acm = self.fetch_acm(timeout=timeout)
        if acm is None:
            return False, "get_planning_scene ACM unavailable"
        for name_a, name_b in pairs:
            set_acm_pair(acm, str(name_a), str(name_b), allowed)
        ok, message = self._apply(build_acm_diff(acm), timeout=timeout)
        if not ok:
            return False, message
        if not verify:
            return True, "acm updated"
        deadline = time.monotonic() + timeout
        names = [(str(a), str(b)) for a, b in pairs]
        while time.monotonic() < deadline:
            live = self.fetch_acm(timeout=min(1.0, timeout))
            if live is not None and all(
                    acm_pair_allowed(live, a, b) == bool(allowed)
                    for a, b in names):
                return True, "acm verified"
            time.sleep(0.1)
        return False, "acm verify timed out"

    def allow_all_collisions(self, timeout=5.0):
        """Site operator override: permit robot/self/world contacts in MoveIt.

        Defaults cover objects added after this call; the explicit matrix is
        updated too because its entries take precedence over defaults.
        """
        scene = self.get_scene(
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
            timeout=timeout)
        if scene is None:
            return False, "collision override: planning scene unavailable"
        acm = scene.allowed_collision_matrix
        names = set(acm.entry_names)
        names.update(obj.id for obj in scene.world.collision_objects)
        names.update(obj.object.id for obj in scene.robot_state.attached_collision_objects)
        names.update({BOX_OBJECT_ID, DEFAULT_ATTACH_LINK, "elfin_base_link"})
        names.update("elfin_link%d" % i for i in range(7))
        for name in sorted(names):
            if name not in acm.default_entry_names:
                acm.default_entry_names.append(name)
                acm.default_entry_values.append(True)
            else:
                acm.default_entry_values[list(acm.default_entry_names).index(name)] = True
        width = len(acm.entry_names)
        for entry in acm.entry_values:
            entry.enabled = [True] * width
        ok, message = self._apply(build_acm_diff(acm), timeout=timeout)
        return ok, "all MoveIt scene collisions allowed" if ok else message
