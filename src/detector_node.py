#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import rospy, rospkg
import message_filters, tf
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

import detector

class DetectorNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = detector.Detector()
        self.broadcaster = tf.TransformBroadcaster()

        rospack = rospkg.RosPack()
        cascade_path = rospack.get_path('mouth_detector')+"/src/haarcascade_frontalface_default.xml"
        self.detector.loadCascade(cascade_path)
        if self.detector.hasEmptyCascade():
            raise IOError('Unable to load the face cascade classifier xml file')

        # rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        image_sub = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image) 
        point_cloud_sub = message_filters.Subscriber("/kinect2/qhd/points", PointCloud2)

        synch_sub = message_filters.TimeSynchronizer([image_sub, point_cloud_sub], 10)
        synch_sub.registerCallback(self.callback)

        self.prev_coords = [(0, 0),(1, 1)] # (x, y) of top left
        self.loaded = True
        self.debug = False

    def callback(self, image, point_cloud=None):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.detector.loadFrameFindFace(self.frame)
        if self.detector.faceIsFound():
            x1, y1, x2, y2 = self.detector.getFaceBox()
            px1, py1 = self.prev_coords[0]
            px2, py2 = self.prev_coords[1]
            if self.debug:
                print("curr: ", x1, y1, x2, y2)
                print("prev: ", px1, py1, px2, py2)
            if self.loaded and (abs(x1-px1) > self.frame.shape[0] / 8 or abs(y1-py1) / 8 > self.frame.shape[1] or (x2-x1) / 2 > (px2-px1) or (y2-y1) / 2 > (py2-py1) or (x2-x1) * 2 < (px2-px1) or (y2-y1) * 2 < (py2-py1)):
                # If the face box changes sizes too quickly or jumps too much, compared to the previous box, then we keep the previous box.
                x1, y1 = self.prev_coords[0]
                x2, y2 = self.prev_coords[1]
            else:
                self.prev_coords[0] = (x1, y1)
                self.prev_coords[1] = (x2, y2)
            cv2.rectangle(self.frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

            mouth_x, mouth_y = ((x1+x2)/2, y1/5+4*y2/5) # Estimated position of mouth
            cv2.circle(self.frame, (mouth_x, mouth_y), (x2-x1)/6, (0, 0, 255), 2)

            cloud_out = pc2.read_points(point_cloud, ("x", "y", "z"), True, [(mouth_x, mouth_y)])
            try:
                mouth_x, mouth_y, mouth_z = next(cloud_out)
            except Exception:
                mouth_x = mouth_y = mouth_z = None
            if mouth_x:
                self.transform_broadcast(mouth_x, mouth_y, mouth_z, image.header.stamp, image.header.frame_id)

        cv2.imshow('Face Detector', self.frame)
        key = cv2.waitKey(1)
        if key == ord('l'):
            self.loaded = True
        elif key == ord('u'):
            self.loaded = False
        elif key == ord('d'):
            self.debug = True
        elif key == ord('q'):
            rospy.signal_shutdown("q was pressed")

    def transform_broadcast(self, x, y, z, timestamp, source_frame):
        # sends out tf
        self.broadcaster.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0), timestamp, "face", source_frame)


if __name__ == '__main__':
    node = DetectorNode()
    rospy.init_node('detector_node')
    rospy.spin()
