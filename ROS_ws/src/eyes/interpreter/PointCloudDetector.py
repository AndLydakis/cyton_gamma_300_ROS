#!/usr/bin/env python

__author__ = 'bencecserna'

import time
import numpy as np
from operator import add, itemgetter
import math
import bisect

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import rospy

global publisher
dim_x = 640
dim_y = 480
dim_z = 3
size = dim_x * dim_y
threshold = 25


def to1D(dim2):
    return dim2[1] * dim_x + dim2[0]


def processCloud(cloud):
    points = pc2.read_points(cloud)
    cloud2 = [None] * size
    # cloud3 = np.ndarray((dim_x, dim_y, 3), dtype=float)
    # cloud3 = cloud3.reshape((size, 3))

    counter = 0
    maxz = 0
    for point in points:
        if point[2] > 0.65:
            z = float('nan')
        else:
            z = point[2]

        cloud2[counter] = (point[0], point[1], z)
        counter += 1

    cloud2 = removeArm(cloud2)

    return pc2.create_cloud_xyz32(cloud.header, cloud2)


def removeArm(cloud):
    mid_x = dim_x / 2
    mid_y = dim_y / 2

    start = time.time()
    # cloud = np.reshape(cloud, (dim_x, dim_y, dim_z))
    mask = fillRemoval(cloud, mid_x, mid_y)
    for i in range(0, size):
        if mask[i]:
            value = 0.7
        else:
            value = cloud[i][2]
        cloud[i] = (cloud[i][0], cloud[i][1], value)

    end = time.time()
    print(end - start)
    return cloud


def fillRemoval(cloud, x, y):
    mask = [0] * size
    mask[to1D((x, y))] = threshold
    open = [(threshold, (x, y))]

    counter = 0
    while open:
        # sorted(open, key=itemgetter(1), reverse=True)
        pos = open.pop()[1]
        mask_value = mask[to1D(pos)] - 1
        counter += 1

        if not math.isnan(cloud[to1D(pos)][2]):

            update_mask_if_larger(mask, open, map(add, pos, (0, 1)), mask_value + 1)
            update_mask_if_larger(mask, open, map(add, pos, (0, -1)), mask_value + 1)
            update_mask_if_larger(mask, open, map(add, pos, (1, 0)), mask_value + 1)
            update_mask_if_larger(mask, open, map(add, pos, (-1, 0)), mask_value + 1)

        elif mask_value is not 0:
            update_mask_if_larger(mask, open, map(add, pos, (0, 1)), mask_value)
            update_mask_if_larger(mask, open, map(add, pos, (0, -1)), mask_value)
            update_mask_if_larger(mask, open, map(add, pos, (1, 0)), mask_value)
            update_mask_if_larger(mask, open, map(add, pos, (-1, 0)), mask_value)

        # if not counter % 10000:
        #     print("...iterations: " + str(counter))

    print("Number of iterations: " + str(counter))

    return mask


def update_mask_if_larger(mask, open, pos, mask_value=threshold):
    if (0 > pos[0]) or (pos[0] >= dim_x) or (0 > pos[1]) or (pos[1] >= dim_y):
        return

    if mask_value > mask[to1D(pos)]:
        mask[to1D(pos)] = mask_value  # Mask of neighbour cell
        bisect.insort(open, (mask_value, pos))


def callback(data):
    rospy.loginfo("PointCloud2 msg received")
    cloud = processCloud(data)
    publisher.publish(cloud)


def listener():
    global publisher
    rospy.loginfo(" LISTENER STARTED")
    print("LISTENER STARTED")
    rospy.init_node('PointCloudDetector')
    rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
    publisher = rospy.Publisher("/segmented_cloud", PointCloud2)
    rospy.spin()


if __name__ == '__main__':
    listener()