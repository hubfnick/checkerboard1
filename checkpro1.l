#!/usr/bin/env roseus

;; robotの初期化
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)

;; ObjectDetection型トピックを使うため
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")

;;; 表示モデルなど
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(objects (list *target-object* *dxl-armed-turtlebot*))

(setq *tfb* (instance ros::transform-broadcaster :init))

(setq found-obj nil)
(setq obj-pos #f(0 0 0))

(ros::roseus "objectdetection_client")

;; ObjectDetection用コールバック関数定義
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
      (ros::ros-info "~A ~A ~A" ret frame-id type)
      (ros::publish "image_marker" mrk))
    ;;(print "~A" obj-pos)
    obj-pos
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "/camera/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(ros::ros-info "get started")
(setq found-obj nil)
(until found-obj
       (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
       (ros::spin-once)
       (ros::sleep)
  )

(ros::ros-warn "found target ojbect ~A, appoaching" obj-pos)

(setq 2d-pos (float-vector (elt obj-pos 0) (elt obj-pos 1)))

(send *ri* :go-pos
(elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 0)
(elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 1)
(rad2deg (atan  (elt obj-pos 1) (elt obj-pos 0))))

;; open gripper
(ros::ros-info "here")
(send *ri* :stop-grasp)


(setq found-obj nil)
(until found-obj
       (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
       (ros::spin-once)
       (ros::sleep)
       )

(ros::ros-warn "found target object ~A" obj-pos)
(setq target-cds (make-coords :pos (scale 1000 obj-pos)))
(send target-cds :translate #f(-200 0 50));;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.

(send target-cds :translate #f(-1800 0 900))
(objects (list *dxl-armed-turtlebot* target-cds))

(send *dxl-armed-turtlebot* :angle-vector #f(0 0 -90 0 90 0))

(send *dxl-armed-turtlebot* :arm :end-coords :translate #f(0 30 0)) ;; change the end-effector position

(send *dxl-armed-turtlebot* :inverse-kinematics target-cds :rotation-axis :y)

#|
(send *dxl-armed-turtlebot* :angle-vector (map float-vector #'(lambda(ang)
								(cond
								 ((> ang 90) (- ang 180))
								 ((< ang -90) (+ ang 180))
								 (t ang)
								 ))
					       (send *dxl-armed-turtlebot* :angle-vector)))
|#

       (ros::ros-warn "the grap arm pose is ~A" (send *dxl-armed-turtlebot* :angle-vector))

       (send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 500)
       (send *ri* :wait-interpolation)

       (send *ri* :go-pos 0.1 0 0)

       ;; grasp
       ;; still some bug in start-grasp, so we have to directly use :move-gripper
       (send *ri* :move-gripper -30 :tm 1500 :wait t)
       (send *ri* :wait-interpolation)

       (send *ri* :angle-vector #f(80 0 45 0 45 45)) ;;  arm tuck pose
       (send *ri* :go-pos -0.6 0 0) ;;  arm tuck pose
