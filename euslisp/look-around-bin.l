#!/usr/bin/env roseus
;; vim: set ft=lisp:

(ros::load-ros-manifest "label_octomap")

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")
(require "package://label_octomap/euslisp/lib.l")


(defun setup ()
  (ros::advertise (format nil "~a/output/start" (ros::get-name)) std_msgs::Empty)
  (ros::advertise (format nil "~a/output/stop" (ros::get-name)) std_msgs::Empty)
  (unix::sleep 1)
  (unless (boundp '*ri*)
    (jsk_2016_01_baxter_apc::baxter-init)
    (send *ri* :gripper-servo-on)
    ; (objects (list *baxter*))
    (send *ri* :recognize-bin-boxes)
    )
  )


;; __main__
(ros::roseus "look_around_bin" :anonymous nil)
(setup)
(look-around-bin :bin :f)
;; untuck-pose
(send *baxter* :untuck-pose :rarm)
(model2real)
;; sleep for octomap topic to save with rosbag
(unix::sleep 3)
(exit)
