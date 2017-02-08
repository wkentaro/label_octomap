#!/usr/bin/env python

import time

import rospy
import std_msgs.msg
import std_srvs.srv


class CallPassthroughThreeViews(object):

    def __init__(self):
        # n_moved:
        #    1: shelf_bin_entrance
        #    2 and 2 seconds: view1
        #    3 and 2 seconds: view2
        #    4 and 2 seconds: view3
        #    5: untuck_pose
        self.n_moved = 0

        self._sub_start = rospy.Subscriber(
            '/look_around_bin_main/output/start',
            std_msgs.msg.Empty, self._cb_move_start)
        self._sub_stop = rospy.Subscriber(
            '/look_around_bin_main/output/stop',
            std_msgs.msg.Empty, self._cb_move_stop)

        self._caller_start = rospy.ServiceProxy('~start', std_srvs.srv.Empty)
        self._caller_stop = rospy.ServiceProxy('~stop', std_srvs.srv.Empty)

    def _cb_move_start(self, msg):
        self._caller_stop.call()

    def _cb_move_stop(self, msg):
        self.n_moved += 1
        if self.n_moved in [2, 3, 4]:
            self._caller_start.call()
        assert self.n_moved <= 5

    # def spin(self):
    #     while not rospy.is_shutdown():
    #         if self._be_called.get(self.n_moved, False):
    #             self._caller_start.call()
    #             time.sleep(2)
    #             self._caller_stop.call()
    #             self._be_called[self.n_moved] = False


if __name__ == '__main__':
    rospy.init_node('call_passthrough_three_views')
    caller = CallPassthroughThreeViews()
    rospy.spin()
