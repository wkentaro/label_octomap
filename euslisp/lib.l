;; vim: set ft=lisp:

(ros::load-ros-manifest "label_octomap")


(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/util.l")


(defun model2real (&key (tm 3000) (wait? t))
  (publish-move :startp t)
  (send *ri* :angle-vector (send *baxter* :angle-vector) tm)
  (when wait?
    (send *ri* :wait-interpolation)
    (publish-move :startp nil)
    )
  )


(defun request-and-stop-pointcloud (&key id)
  (unix:usleep (* 250 1000))
  (ros::service-call "/passthrough_cloud/request" (instance std_srvs::EmptyRequest :init))
  (ros::service-call
    (format nil "/passthrough_viewpoint_~a/request" id)
    (instance std_srvs::EmptyRequest :init))
  ; (unix:usleep (* 1000 1000))
  (unix:sleep 1)
  (ros::service-call "/passthrough_cloud/stop" (instance std_srvs::EmptyRequest :init))
  (ros::service-call
    (format nil "/passthrough_viewpoint_~a/stop" id)
    (instance std_srvs::EmptyRequest :init))
  (unix:usleep (* 250 1000))
  )


(defun look-around-bin (&key (bin :f) (times 3) (do-sleep t))
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
  (when do-sleep (unix:sleep 5))
  ;; center
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 4) :y :world)
  (send tc :translate (float-vector 30 0 50) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 1)
  (when do-sleep (unix:sleep 5))
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
  (when do-sleep (unix:sleep 5))

  (when (not (equal times 6)) (return-from 'look-around-bin))

  ;; pi/9 rotation in y axis
  ;; left
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 9) :y :world)
  (send tc :rotate (- (/ pi 12)) :z :world)
  (send tc :translate (float-vector 30 100 10) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 3)
  (when do-sleep (unix:sleep 5))
  ;; center
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 9) :y :world)
  (send tc :translate (float-vector 30 0 10) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 4)
  (when do-sleep (unix:sleep 5))
  ;; right
  (setq tc (send init-tc :copy-worldcoords))
  (send tc :rotate (/ pi 9) :y :world)
  (send tc :rotate (/ pi 12) :z :world)
  (send tc :translate (float-vector 30 -100 10) :world)
  (send *baxter* :rarm :inverse-kinematics tc
        :rthre (/ pi 18)
        )
  (model2real)
  (request-and-stop-pointcloud :id 5)
  (when do-sleep (unix:sleep 5))
  )


(defun publish-move (&key startp)
  (let ((msg (instance std_msgs::Empty :init)))
    (if startp
      (ros::publish (format nil "~a/output/start" (ros::get-name)) msg)
      (ros::publish (format nil "~a/output/stop" (ros::get-name)) msg)
      )
    )
  )


(defun baxter-rarm-only-pose ()
  (send *baxter* :larm :angle-vector
        #f(41.9897 1.03271 5.95459 41.4404 -13.7988 40.188 7.29492 0.0))
  )
