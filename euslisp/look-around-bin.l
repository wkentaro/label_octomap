#!/usr/bin/env roseus
;; vim: set ft=lisp:

(ros::load-ros-manifest "icra2017")

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")
(require "package://icra2017/euslisp/lib.l")


(defun setup ()
  (unless (boundp '*ri*)
    (jsk_2016_01_baxter_apc::baxter-init)
    (send *ri* :gripper-servo-on)
    ; (objects (list *baxter*))
    (send *ri* :recognize-bin-boxes)
    )
  (ros::advertise (format nil "~a/output/moved" (ros::get-name)) std_msgs::Empty)
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
