#!/usr/bin/env python3
import logging
import rospy
import geometry_msgs
import tf2_ros

class TFBroadCaster(object):

    logger = logging.getLogger(__name__)
    logger.setLevel('DEBUG')

    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def broadcast(self, target):
        if not rospy.is_shutdown():
            self.logger.info('pose estimation delay: {}'.format(rospy.Time.now().to_time() - target.camera_image.timestamp))
            self.broadcaster.sendTransform(self.build_tf(target))

    def build_tf(self, detection_object):
        t = geometry_msgs.msg.TransformStamped()
        [x, y, z] = detection_object.position
        [qw, qx, qy, qz] = detection_object.quaternion

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = detection_object.camera_image.camera.frame_id
        t.child_frame_id = detection_object.get_unique_ar_id_string()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t
