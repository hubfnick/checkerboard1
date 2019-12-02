#!/usr/bin/env roseus

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)


(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
;;(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(objects (list *target-object* *dxl-armed-turtlebot*))

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq countsave 0)
(ros::rate 100)



(defun objectdetection-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) frame-id type ret)
    ;; 物体モデルを配置
    (setq frame-id (concatenate string "/" (send msg :header :frame_id)))
    (mapcar #'(lambda (obj-pose)
                (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *dxl-armed-turtlebot* :camera_rgb_optical_frame_lk) :copy-worldcoords)))
		  (setq obj-pos (scale 0.001 (send (send cam-coords :transform cam->obj-coords) :worldpos)))
		  (setq found-obj t)
		  (setq countsave 0)
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
                  (send *target-object* :newcoords (send cam-coords :transform cam->obj-coords))
		  
		  
                  ))
	    (send msg :objects))
    
    ;; image_markerを出力
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list type))
      (send *tfb* :send-transform ret frame-id type)
      ;;(ros::ros-info "~A ~A ~A" ret frame-id type)
      (ros::publish "image_marker" mrk))
    ;;(print "~A" obj-pos)
    obj-pos
    ))
	    

(setq found-obj nil)
(setq obj-pos #f(0 0 0))

(ros::roseus "tf-listener")
(setq timestate 0)
(setq timechange t)
(setq found-obj nil)
(setq obj-pos #f(0 0 0))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "/camera/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(setq termstart (ros::time-now))
(do-until-key
 (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
(send target-cds :translate #f(-200 0 50));;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.

(send target-cds :translate #f(700 0 -300))
 (setq curtime (ros::time-now))
 (if (ros::time>= (ros::time- curtime termstart) (ros::time 0.8))
     (progn
       (format t "elapsed ~A sec ~%" (send (ros::time- (ros::time-now) termstart) :to-sec))
       (setq termstart curtime)
       (setq timestate (mod (+ timestate 1) 4))
       (setq timechange t)
       
       )
   )
 (if timechange
     (progn
       (send *ri* :publish-sound :error)
       (cond ((= timestate 0) (progn
				(print "0")
				
				(if found-obj
				    (progn
				      (send *dxl-armed-turtlebot* :inverse-kinematics target-cds :rotation-axis :y :revert-if-fail nil)
				      (send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 300)
				      (send *ri* :wait-interpolation)
				      (print "inverse")
				      )
				  (print "no obj found")
				  )
				;;(send *ri* :angle-vector #f(0 0 -50 0 50 0) 1500)
				;;(send *ri* :wait-interpolation)
				))
	     ((= timestate 1) (progn
				(print "1")
				(send *ri* :angle-vector #f(0 0 -50 0 50 0) 300)
				(send *ri* :wait-interpolation)
				))
	     ((= timestate 2) (progn
				(print "2")
				))
	     ((= timestate 3) (progn
				(print "3")
				)))
       (setq timechange nil)
       )
   )
(setq countsave (+ 1 countsave))
(if (> countsave 5)
    (setq found-obj nil)
  )
 (ros::spin-once)
 (ros::sleep)
 )
