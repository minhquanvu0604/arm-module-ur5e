#!/usr/bin/env python

import rospy
import tf.transformations as tf_trans

from arm_module_ur5e.srv import PoseService, PoseServiceRequest


def query_pose():
    rospy.init_node('pose_query_node')
    rospy.wait_for_service('/mvps/arm_module/query_data')

    try:
        query_service = rospy.ServiceProxy('/mvps/arm_module/query_data', PoseService)
        request = PoseServiceRequest()
        response = query_service(request)

        position = response.pose.pose.position
        orientation = response.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)

        rospy.loginfo("Received pose data:\n \
                    Position: x=%.2f, y=%.2f, z=%.2f\n \
                    Orientation: roll=%.2f, pitch=%.2f, yaw=%.2f",
                    position.x, position.y, position.z, roll, pitch, yaw)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        query_pose()
    except rospy.ROSInterruptException:
        pass
