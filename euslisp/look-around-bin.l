#!/usr/bin/env roseus
;; vim: set ft=lisp:

(ros::load-ros-manifest "icra2017")

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")


(defun model2real (&key (tm 3000) (wait? t))
  (send *ri* :angle-vector (send *baxter* :angle-vector) tm)
  (when wait?
    (send *ri* :wait-interpolation)
    )
  )


(defun setup ()
  (unless (boundp '*ri*)
    (jsk_2016_01_baxter_apc::baxter-init)
    (send *ri* :gripper-servo-on)
    (objects (list *baxter*))
    (send *ri* :recognize-bin-boxes)
    )
  )


(defun request-and-stop-pointcloud
  ()
  (unix:usleep (* 250 1000))
  (ros::service-call "/passthrough_cloud/request" (instance std_srvs::EmptyRequest :init))
  (unix:usleep (* 1000 1000))
  (ros::service-call "/passthrough_cloud/stop" (instance std_srvs::EmptyRequest :init))
  (unix:usleep (* 250 1000))
  )


(defun look-around-bin (&key (bin :f))
  "Look around bin with 6 view points"
  (send *ri* :ik->bin-entrance :rarm bin
        :gripper-angle 90
        :offset (float-vector -200 0 0))
  (model2real)

  ;; initial pose
  (send *baxter* :head_pan :joint-angle 70)
  (send *ri* :ik->bin-entrance :rarm bin
        :gripper-angle 90
        :offset (float-vector -200 0 -80))
  (setq init-tc (send (send *baxter* :rarm :end-coords) :copy-worldcoords))

  ;; pi/4 rotation in y axis
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :rotate (- (/ pi 12)) :z :world)
  (send tc :translate (float-vector 30 100 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud)
  ;;
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :translate (float-vector 30 0 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud)
  ;;
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :rotate (/ pi 12) :z :world)
  (send tc :translate (float-vector 30 -100 0) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud)
  )


;; __main__
(ros::roseus "look_around_bin" :anonymous nil)
(setup)
(look-around-bin :bin :f)
(exit)
