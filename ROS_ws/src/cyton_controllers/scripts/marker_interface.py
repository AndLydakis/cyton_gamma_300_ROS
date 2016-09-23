#!/usr/bin/env python
__author__ = 'Bence Cserna'

import rospy
import tf
import numpy as np
import time
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers

entities = {}
IIR_ALPHA = 0.6
WINNOW_ALPHA = 1.5
EPSILON = 0.1
CONFIDENCE_LOW = 0.1
CONFIDENCE_HIGH = 0.9
CONFIDENCE_LIMIT = 0.8

PARENT_FRAME = "camera_rgb_optical_frame"
ARM_BASE = "base_link"

base_position = None
broadcaster = tf.TransformBroadcaster()

# noise = open("/home/ros/noise", "w")
# filter = open("/home/ros/filter", "w")


class Entity:
    def __init__(self, id):
        self.id = id
        self.position = {}
        self.orientation = {}
        self.confidence = 0
        self.is_initialized = False

    def update(self, position, orientation):
        if not self.is_initialized:
            self.position = position
            self.orientation = orientation
            self.is_initialized = True
        else:
            diff_vector = [abs(old - new) for old, new in zip(self.position, position)]
            magnitude = np.linalg.norm(diff_vector)
            if magnitude < EPSILON:
                self.confidence *= WINNOW_ALPHA
            else:
                self.confidence /= WINNOW_ALPHA

            if self.confidence < CONFIDENCE_LOW:
                self.confidence = CONFIDENCE_LOW
            elif self.confidence > CONFIDENCE_HIGH:
                self.confidence = CONFIDENCE_HIGH

            alpha = (1 - self.confidence) * IIR_ALPHA

            # IIR Filtering
            self.position = iir_filter(alpha, self.position, position)
            self.orientation = iir_filter(alpha, self.orientation, orientation)

        # if self.id is 2:
        #     filter.write(str(self.orientation[0]) + "\n")
        #     noise.write(str(orientation[0]) + "\n")

        rospy.loginfo("Filtered " + str(self.id))
        broadcaster.sendTransform(self.position,
                                  self.orientation,
                                  rospy.Time.now(),
                                  "filtered_" + str(self.id),
                                  PARENT_FRAME)


def iir_filter(alpha, old_values, new_values):
    return [(1 - alpha) * old + alpha * new for old, new in zip(old_values, new_values)]


def process_tag_data(marker_info):
    id = marker_info.id
    # rospy.loginfo(id)

    # Create new Entity if absent
    if id not in entities:
        entities[id] = Entity(id)
        rospy.loginfo("New entity created with id: " + str(id))

    # Get tag position and orientation
    position = marker_info.pose.pose.position
    orientation = marker_info.pose.pose.orientation
    position = (position.x, position.y, position.z)
    orientation = (orientation.x, orientation.y, orientation.z, orientation.w)

    entity = entities[id]
    # Update tag position
    entity.update(position, orientation)
    #
    # # if entity.confidence > CONFIDENCE_LIMIT:
    # if base_position is not None:
    #     t = geometry_msgs.msg.PoseStamped()
    #     t.header.frame_id = PARENT_FRAME
    #     t.header.stamp = rospy.Time.now()
    #     t.pose.position.x = entity.position[0]# - base_position[0]
    #     t.pose.position.y = entity.position[1]# - base_position[1]
    #     t.pose.position.z = entity.position[2]# - base_position[2]
    #
    #     t.pose.orientation.x = entity.orientation[0]
    #     t.pose.orientation.y = entity.orientation[1]
    #     t.pose.orientation.z = entity.orientation[2]
    #     t.pose.orientation.w = entity.orientation[3]
    #
    #     listener = tf.TransformListener()
    #     # transform = listener.lookupTransform(PARENT_FRAME, ARM_BASE, rospy.Time.now())
    #     tx = listener.transformPose(ARM_BASE, t)
    #     # tx = t
    #     #
    #     br = tf.TransformBroadcaster()
    #     br.sendTransform((tx.pose.position.x, tx.pose.position.y, tx.pose.position.z),
    #     (tx.pose.orientation.x, tx.pose.orientation.y, tx.pose.orientation.z, tx.pose.orientation.w),
    #     rospy.Time.now(),
    #     "now_" + str(id),
    #     ARM_BASE)
    # else:
    #     rospy.loginfo("Base position is null.")


def publish_base_tf():
    corner_positions = zip(entities[0].position, entities[1].position, entities[2].position, entities[3].position)
    corner_orientations = zip(entities[0].orientation, entities[1].orientation, entities[2].orientation,
                              entities[3].orientation)
    base_position = [(q0 + q1 + q2 + q3) / 4 for q0, q1, q2, q3 in corner_positions]
    base_orientation = [(q0 + q1 + q2 + q3) / 4 for q0, q1, q2, q3 in corner_orientations]

    base_orientation = entities[2].orientation

    # Update tf
    broadcaster.sendTransform(base_position,
                              base_orientation,
                              rospy.Time.now(),
                              ARM_BASE,
                              PARENT_FRAME)


def callback(data):
    for tag_data in data.markers:
        process_tag_data(tag_data)
    publish_base_tf()


def listener():
    rospy.loginfo(" LISTENER STARTED")
    print("LISTENER STARTED")
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers,
                     callback)
    # spin() simply keeps python from exiting until this node is stopped
    time.sleep(60)
    # noise.close()
    # filter.close()
    rospy.spin()


if __name__ == '__main__':
    listener()
