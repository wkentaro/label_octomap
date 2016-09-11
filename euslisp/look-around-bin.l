#!/usr/bin/env roseus
;; vim: set ft=lisp:

(ros::load-ros-manifest "icra2017")

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")


(defun setup ()
  (unless (boundp '*ri*)
    (jsk_2016_01_baxter_apc::baxter-init)
    (send *ri* :gripper-servo-on)
    ; (objects (list *baxter*))
    (send *ri* :recognize-bin-boxes)
    )
  (ros::advertise (format nil "~a/output/moved" (ros::get-name)) std_msgs::Empty)
  )


(defun publish-moved ()
  (ros::publish
    (format nil "~a/output/moved" (ros::get-name))
    (instance std_msgs::Empty :init))
  )


(defun model2real (&key (tm 3000) (wait? t))
  (send *ri* :angle-vector (send *baxter* :angle-vector) tm)
  (when wait?
    (send *ri* :wait-interpolation)
    )
  (publish-moved)
  )


(defun request-and-stop-pointcloud (&key id)
  (unix:usleep (* 250 1000))
  (ros::service-call "/passthrough_cloud/request" (instance std_srvs::EmptyRequest :init))
  (ros::service-call
    (format nil "/passthrough_viewpoint_~a/request" id)
    (instance std_srvs::EmptyRequest :init))
  ; (unix:usleep (* 1000 1000))
  (unix:sleep 2)
  (ros::service-call "/passthrough_cloud/stop" (instance std_srvs::EmptyRequest :init))
  (ros::service-call
    (format nil "/passthrough_viewpoint_~a/stop" id)
    (instance std_srvs::EmptyRequest :init))
  (unix:usleep (* 250 1000))
  (unix:sleep 5)  ;; wait for processing
  )


(defun look-around-bin (&key (bin :f))
  "Look around bin with 6 view points"
  (send *ri* :ik->bin-entrance :rarm bin
        :gripper-angle 90
        :offset (float-vector -200 0 0))
  (model2real)

  ;; initial pose
  (send *baxter* :head_pan :joint-angle 70)
  (send *baxter* :fold-pose-back :larm)
  (send *ri* :ik->bin-entrance :rarm bin
        :gripper-angle 90
        :offset (float-vector -200 0 -80))

  (setq init-tc (send (send *baxter* :rarm :end-coords) :copy-worldcoords))

  ;; pi/4 rotation in y axis
  ;; right
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :rotate (/ pi 12) :z :world)
  (send tc :translate (float-vector 30 -100 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 0)
  ;; center
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :translate (float-vector 30 0 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 1)
  ;; left
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :rotate (- (/ pi 12)) :z :world)
  (send tc :translate (float-vector 30 100 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 2)

  ;; untuck-pose
  (send *baxter* :untuck-pose :rarm)
  (model2real)

  ;; sleep for octomap topic to save with rosbag
  (unix::sleep 3)
  )


;; __main__
(ros::roseus "look_around_bin" :anonymous nil)
(setup)
(look-around-bin :bin :f)
(exit)
