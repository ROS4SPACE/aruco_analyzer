#!/usr/bin/env python3
import logging
import rospy
import geometry_msgs
import tf2_ros
from aruco_analyzer.helper_classes.config import Config

class TFBroadCaster(object):

    logger = logging.getLogger(__name__)
    logger.setLevel('DEBUG')

    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.config = Config()

    def broadcast(self, target):
        if not rospy.is_shutdown():
            self.logger.info('pose estimation delay: {}'.format(rospy.Time.now().to_time() - target.camera_image.timestamp))
            self.broadcaster.sendTransform(self.build_tf(target))

            if (target.marker_type == "G"):
                # target is a grid. grids have their tf located at the corner. lets also publish its center.
                self.broadcaster.sendTransform(self.build_grid_center_tf(target))

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

    def build_grid_center_tf(self, target):
        t = geometry_msgs.msg.TransformStamped()

        my_board = None

        # get the size of the grid
        for board in self.config.board_config:
            if board.id == target.get_unique_ar_id_string():
                my_board = board
        
        # TODO: maybe the white surrounding border also must be considered for the offset to the center
        #       it depends on where the corner tf is located exactly (on the white corner or the black one)
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = target.get_unique_ar_id_string()
        t.child_frame_id = target.get_unique_ar_id_string() + "_center"
        if (my_board.x == 1):
            t.transform.translation.x = my_board.marker_length / 2
        else:
            t.transform.translation.x = (my_board.marker_length + my_board.marker_separation) * my_board.x / 2
        if (my_board.y == 1):
            t.transform.translation.y = my_board.marker_length / 2
        else:
            t.transform.translation.y = (my_board.marker_length + my_board.marker_separation) * my_board.y / 2
        t.transform.translation.z = 0
        t.transform.rotation.w = 1

        return t