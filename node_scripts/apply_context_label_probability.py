#!/usr/bin/env python

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image

import numpy as np


class ApplyContextLabelProbability(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        # list of label values
        self.candidates = rospy.get_param('~candidates', [])
        if not all(isinstance(lbl, int) and lbl >= 0
                   for lbl in self.candidates):
            rospy.logfatal("Elements of '~candidates' must be "
                           "integer and >=0.")
            quit(1)
        self.pub_proba = self.advertise('~output', Image, queue_size=1)
        self.pub_label = self.advertise('~output/label', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.apply)

    def unsubscribe(self):
        self.sub.unregister()

    def apply(self, imgmsg):
        bridge = cv_bridge.CvBridge()

        proba_img = bridge.imgmsg_to_cv2(imgmsg).copy()  # copy to change it
        if proba_img.ndim != 3:
            rospy.logerr('Image shape must be (height, width, channels).')
            return

        n_labels = proba_img.shape[2]
        if max(self.candidates) >= n_labels:
            rospy.logerr("The max label value in '~candidates' exceeds "
                         "the number of input labels.")
            return

        for lbl in range(n_labels):
            if lbl not in self.candidates:
                proba_img[:, :, lbl] = 0.

        # do dynamic scaling for the probability image
        proba_img /= np.atleast_3d(proba_img.sum(axis=-1))

        out_proba_msg = bridge.cv2_to_imgmsg(proba_img)
        out_proba_msg.header = imgmsg.header
        self.pub_proba.publish(out_proba_msg)

        label_img = proba_img.argmax(axis=-1).astype(np.int32)
        out_label_msg = bridge.cv2_to_imgmsg(label_img, encoding='32SC1')
        out_label_msg.header = imgmsg.header
        self.pub_label.publish(out_label_msg)


if __name__ == '__main__':
    rospy.init_node('apply_context_label_probablity')
    ApplyContextLabelProbability()
    rospy.spin()
