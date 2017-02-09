#!/usr/bin/env roseus
;; vim: set ft=lisp:

(ros::load-ros-manifest "label_octomap")

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")
(require "package://label_octomap/euslisp/lib.l")


(defun setup ()
  (unless (boundp '*ri*)
    (jsk_2016_01_baxter_apc::baxter-init)
    (send *ri* :gripper-servo-on)
    (baxter-rarm-only-pose)
    (send *baxter* :fold-pose-back :rarm)
    (send *baxter* :rotate-gripper :rarm 90)
    (model2real)
    (send *ri* :calib-pressure-threshold :rarm)
    (send *ri* :recognize-bin-boxes)
    )
  (ros::advertise (format nil "~a/output/start" (ros::get-name)) std_msgs::Empty)
  (ros::advertise (format nil "~a/output/stop" (ros::get-name)) std_msgs::Empty)
  )


(defun is-invalid-pose (pose)
  (let ((invalid nil)
        (q-norm (norm
                  (float-vector
                    (send pose :orientation :x)
                    (send pose :orientation :y)
                    (send pose :orientation :z)
                    (send pose :orientation :w)))))
    (unless (and (<= 0.99 q-norm) (<= q-norm 1.01))
      (setq invalid t)
      )
    invalid))


(defun pick-backward-object (&key target-label)
  (let ((centeroids-msg)
        (coords-table (make-hash-table))
        (tc)  ;; target coordinate
        (non-tcs)  ; non-target coordinates
        )
    (setq centroids-msg
      (one-shot-subscribe
        "/cluster_indices_decomposer_voxels/centroid_pose_array"
        geometry_msgs::PoseArray
        :after-stamp (ros::time-now))
      )
    (dotimes (i (length (send centroids-msg :poses)))
      (unless (eq i 0)  ;; 0 is bg_label and should be skipped
        (let ((frame_id (send centroids-msg :header :frame_id))
              (pose (elt (send centroids-msg :poses) i)))
          (unless (is-invalid-pose pose)
            (sethash i coords-table (send *ri* :tf-pose->coords frame_id pose))
            )
          )
        )
      )

    ;;
    (dolist (kv (send coords-table :list))
      (ros::ros-info-green "Object Coords: label=~a, coords=~a" (car kv) (cdr kv))
      )
    (setq target-label-coords (gethash target-label coords-table))
    (send coords-table :delete target-label)

    ;; remove collision objects
    (dolist (kv (send coords-table :list))
      (let ((label (car kv))
            (coords (cdr kv)))
        ;; other object is located in the front of target object
        (when (or
                (null target-label-coords)
                (< (elt (send coords :worldpos) 0) (elt (send target-label-coords :worldpos) 0)))
          ;; pick
          (ros::ros-info-green "[PICK:~a] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" label)
          (ros::set-dynamic-reconfigure-param "/cluster_indices_to_indices" "index" :int label)
          (send *ri* :recognize-objects-segmentation-in-bin :rarm :f)
          (send *ri* :pick-object :rarm :f :target-offset #f(50 0 0))
          (ros::ros-info-green "[PICK:~a] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" label)
          (unless (send *ri* :graspingp :rarm)
            (pick-backward-object :target-label target-label)
            (return-from 'pick-backward-object nil)
            )
          ;; place
          (ros::ros-info-green "[PLACE SHELF:~a] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" label)
          (send *ri* :move-arm-body->bin :rarm :i)
          (send *ri* :wait-interpolation)
          (send *ri* :ik->bin-entrance :rarm :i :offset (float-vector -100 0 30))
          (model2real)
          (send *ri* :ik->bin-entrance :rarm :i :offset (float-vector 200 0 30))
          (model2real)
          (send *ri* :stop-grasp :rarm)
          (unix:sleep 3)
          (send *ri* :ik->bin-entrance :rarm :i :offset (float-vector -150 0 30))
          (model2real)
          (ros::ros-info-green "[PLACE SHELF:~a] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" label)
          ;; finalize loop
          (send coords-table :delete label)
          (when (null target-label-coords)
            (pick-backward-object :target-label target-label)
            (return-from 'pick-backward-object nil)
            )
          )
        )
      )

    ;; pick target object
    (ros::ros-info-green "[PICK:~a] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" target-label)
    (ros::set-dynamic-reconfigure-param "/cluster_indices_to_indices" "index" :int target-label)
    (send *ri* :recognize-objects-segmentation-in-bin :rarm :f)
    (send *ri* :pick-object :rarm :f
          :target-offset #f(50 0 0))
    (ros::ros-info-green "[PICK:~a] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" target-label)
    (ros::ros-info-green "[PLACE ORDER:~a] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" target-label)
    (send *ri* :place-object :rarm)
    (ros::ros-info-green "[PLACE ORDER:~a] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" target-label)
    )
  )


;; __main__
(ros::roseus "pick_backward_object" :anonymous nil)
(setup)
(look-around-bin :bin :f :times 6 :do-sleep nil)
(setq target-label
      (floor (ros::get-param (format nil "~a/target_label" (ros::get-name)))))
(ros::ros-info-green "Target object label is ~a." target-label)
(pick-backward-object :target-label target-label)
(exit)
