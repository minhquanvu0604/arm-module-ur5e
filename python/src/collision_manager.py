#! /usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped

from python.src.utility import *


class CollisionManager:
    r"""
    A class to manage the collision objects in the scene.
    """

    def __init__(self, scene: PlanningSceneInterface) -> None:

        self._scene = scene
        self.bottles = []


    def add_crate_bound(self):
        r"""
        add a thin box above the crate to restraint the robot motion not to go downward too much
        """

        bound_pose = PoseStamped()
        bound_pose.pose = list_to_pose([0.0, -0.36, -0.17, 0.0, 0.0, 0.0])
        bound_pose.header.frame_id = "base_link"
        bound_id = "bound"
        self._scene.add_box(bound_id, bound_pose, size=(0.45, 0.35, 0.01))   

        return bound_id
    
    def remove_crate_bound(self, bound_id):
        r"""
        remove the thin box above the crate
        """
        if self._scene.get_objects(bound_id) is not None:
            self._scene.remove_world_object(bound_id[0])

    def add_abitrary_box(self, pose: Pose, obj_ID: str, frame_id: str, size) -> tuple:

        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose = pose

        self._scene.add_box(obj_ID, obj_pose, size=size)

        return self.wait_for_obj_state(obj_name=obj_ID, obj_is_known=True, obj_is_attached=False), obj_ID

    def add_collision_object(self, obj_id : str, pose: Pose, object_type: str, frame_id: str) -> tuple:

        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose = pose

        mesh_path = f"package://src/ur3e_controller/meshes/{object_type}.stl"
        self._scene.add_mesh(obj_id, obj_pose, filename=mesh_path)

        return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True), obj_id

    def add_box_collision_object(self, pose: Pose, object_type: str, frame_id: str, size: tuple) -> tuple:

        if object_type not in self._object_counter:
            raise ValueError(f"Invalid object name: {object_type}")

        self._object_counter[object_type] += 1
        obj_id = f"{object_type}_{self._object_counter[object_type]:02d}"
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose = pose

        self._scene.add_box(obj_id, obj_pose, size=size)
        return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True, obj_is_attached=True), obj_id
    
    def add_cylinder(self, pose: Pose, object_type: str, frame_id: str, height:float, radius:float) -> tuple:
        if object_type not in self._object_counter:
            raise ValueError(f"Invalid object name: {object_type}")

        self._object_counter[object_type] += 1
        obj_id = f"{object_type}_{self._object_counter[object_type]:02d}"
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose = pose

        self._scene.add_cylinder(obj_id, obj_pose, height, radius)
        return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True, obj_is_attached=True), obj_id

    def add_cone_collision_object(self, pose: Pose, object_type: str, frame_id: str, size: tuple) -> tuple:

        if object_type not in self._object_counter:
            raise ValueError(f"Invalid object name: {object_type}")

        self._object_counter[object_type] += 1
        obj_id = f"{object_type}_{self._object_counter[object_type]:02d}"
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose = pose

        self._scene.add_cone(obj_id, obj_pose, height=size[0], radius=size[1])
        return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True, obj_is_attached=True), obj_id

    def remove_collision_object(self, obj_id=None) -> tuple:

        if obj_id is None:
            self._scene.remove_world_object()
        else:
            self._scene.remove_world_object(obj_id)

    def attach_object(self, eef_link, obj_id, touch_links=None ):

        self._scene.attach_mesh(eef_link, obj_id, touch_links=touch_links)
        return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True, obj_is_attached=True)

    def detach_object(self, eef_link=None, obj_id=None):

        self._scene.remove_attached_object(link=eef_link, name=obj_id)
        if obj_id is not None:
            return self.wait_for_obj_state(obj_name=obj_id, obj_is_known=True, obj_is_attached=False)
        else:
            return True

    def wait_for_obj_state(self, obj_name: str, obj_is_known: bool, obj_is_attached: bool, timeout: float = 4) -> bool:

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = self._scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = obj_name in self._scene.get_known_object_names()

            if (obj_is_attached == is_attached) and (obj_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_bottle(self, initial_pose) -> str:
        r"""
        Add a bottle to the scene and store it in the list.

        :param initial_pose: The initial pose of the bottle.
        :type initial_pose: Pose
        :return: The ID of the bottle.
        """

        bottle = Bottle(
            id=f"bottle_{len(self.bottles)}", type="", pose=initial_pose)
        self.bottles.append(bottle)

        object_is_added, id =self.add_collision_object(obj_id=bottle.id, pose=initial_pose,
                                  object_type="bottle", frame_id="base_link_inertia")

        return object_is_added, bottle.id 

    def update_bottle_type(self, bottle_id, new_type) -> None:
        r"""
        Update the type of the bottle.

        :param bottle_id: The ID of the bottle.
        :type bottle_id: str
        :param new_type: The new type of the bottle.
        :type new_type: str
        """

        for bottle in self.bottles:
            if bottle.id == bottle_id:
                bottle.type = new_type
                break
