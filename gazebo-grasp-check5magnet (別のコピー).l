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
(load "models/chessboard-10-3x3-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(setq *target-object2* (chessboard-10-3x3 :name "/test_object2"))
(objects (list *target-object* *target-object2* *dxl-armed-turtlebot*))

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfb2* (instance ros::transform-broadcaster :init))
(setq countsave 0)
(setq countsave2 0)
(ros::rate 100)
(setq magnetflag 0)
(setq objstate 0)

(setq msg (instance std_msgs::Int32 :init))

;;(send *ri* :publish-sound :error)


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
		  (orient-coords-to-axis *target-object* #f(0 0 1))
		  
		  
		  
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

(defun objectdetection-cb2 (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) frame-id type ret)
    ;; 物体モデルを配置
    (setq frame-id2 (concatenate string "/" (send msg :header :frame_id)))
    (mapcar #'(lambda (obj-pose)
                (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *dxl-armed-turtlebot* :camera_rgb_optical_frame_lk) :copy-worldcoords)))
		  (setq obj-pos2 (scale 0.001 (send (send cam-coords :transform cam->obj-coords) :worldpos)))
		  (setq found-obj2 t)
		  (setq countsave2 0)
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
                  (send *target-object2* :newcoords (send cam-coords :transform cam->obj-coords))
		  (orient-coords-to-axis *target-object2* #f(0 0 1))
		  
                  ))
	    (send msg :objects))
    
    ;; image_markerを出力
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list type))
      (send *tfb2* :send-transform ret frame-id type)
      ;;(ros::ros-info "~A ~A ~A" ret frame-id type)
      (ros::publish "image_marker2" mrk))
    ;;(print "~A" obj-pos)
    obj-pos2
    ))


(setq found-obj nil)
(setq found-obj2 nil)
(setq obj-pos #f(0 0 0))
(setq obj-pos2 #f(0 0 0))
(setq yetstart t)

(ros::roseus "tf-listener")
(setq timestate 0)
(setq postoput #f(0 0 0))
(setq found-obj nil)


(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::advertise "image_marker2" image_view2::ImageMarker2 1)
(ros::advertise "magnetflag" std_msgs::Int32 1)
(ros::subscribe "/camera/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)
(ros::subscribe "/camera/rgb/ObjectDetection2" posedetection_msgs::ObjectDetection #'objectdetection-cb2)
(send *ri* :angle-vector #f(0 0 -20 0 50 0) 1000)
(send *ri* :wait-interpolation)
(send *dxl-armed-turtlebot* :angle-vector (send *ri* :state :potentio-vector))

(setq timechange t)
(do-until-key
 (send *dxl-armed-turtlebot* :arm :end-coords :draw-on :flush t :size 100)
 (if yetstart 
     (progn (print "found1")
	    (print found-obj)
	    (print "found2")
	    (print found-obj2)
	    (setq termstart (ros::time-now))

	    (if (and found-obj found-obj2) (setq yetstart nil)))
   

   (progn
     (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
     (print (target-cds :rot))
     (setq target-cds2 (make-coords :pos (scale 1000 obj-pos2)))
     (print (target-cds2 :rot))
     (send target-cds :translate #f(-200 0 50));;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
     (send target-cds2 :translate #f(-200 0 50))
     (send target-cds2 :translate #f(0 0 10))
     (send target-cds2 :translate #f(0 0 10))
     ;;(send target-cds :translate #f(200 0 -100))
     ;;(send target-cds2 :translate #f(200 0 -100))
     (setq curtime (ros::time-now))
     (if (ros::time>= (ros::time- curtime termstart) (ros::time 0.8))
	 (progn
	   (format t "elapsed ~A sec ~%" (send (ros::time- (ros::time-now) termstart) :to-sec))
	   (setq termstart curtime)
	   (setq timestate (mod (+ timestate 1) 4))
	   (setq timechange t)
	   
	   )
       )
     (if (ros::time>= (ros::time- curtime termstart) (ros::time 0.4))
	 ;;(send *ri* :publish-sound :off)
       )
     (if timechange
	 (progn
	   (send *ri* :publish-sound :on)
	   (cond ((= timestate 0) (progn
				    (print "0")
				    (if (eql magnetflag 1)
					(progn
					  
					  (setq target-cds (make-coords :pos (scale 1000 postoput)))
					  (print (target-cds :rot))
					  (print obj-pos)
					  (print postoput)
					  
					  (send target-cds :translate #f(-200 0 50))
					  (send target-cds :translate #f(0 0 10))
					  (send *dxl-armed-turtlebot* :inverse-kinematics target-cds :revert-if-fail t :rotation-axis :z)
					  ;;(send *dxl-armed-turtlebot* :inverse-kinematics (send *target-object* :copy-worldcoords) :revert-if-fail t :rotation-axis :z)
							     (send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 300)
							     (send *ri* :wait-interpolation)
							     ;;(send *ri* :publish-sound :error)
							     (print "inverse")
							     (setq magnetflag 0)
							     )
				      
				      ;;(send target-cds :translate #f(200 0 -100));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				      (setq savecoord (send *target-object* :copy-worldcoords));;;;;;;;;
				      (if found-obj
					  (progn
					    ;;(send *ri* :publish-sound :error)
					    
					    
					    (setq magnetflag (mod (+ 1 magnetflag) 2))
					    (send msg :data magnetflag)
					    
					    
					    
					    (ros::publish "magnetflag" msg)
					    (if (eql magnetflag 1) (setq postoput obj-pos))
					    
					    ;;(send *ri* :publish-sound :error)
					    ;;(if (eql objstate 1) (setq postoput target-cds))
					    
					    (send *dxl-armed-turtlebot* :inverse-kinematics target-cds :revert-if-fail t :rotation-axis :z)
					    ;;(send *dxl-armed-turtlebot* :inverse-kinematics (send *target-object2* :copy-worldcoords) :revert-if-fail t :rotation-axis :z)
					    (send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 300)
					    (send *ri* :wait-interpolation)
					    ;;(send *ri* :publish-sound :error)
					    (print "inverse")
					    )
					(print "no obj found")
					)
				      )))
				      
				      ;;(send *ri* :angle-vector #f(0 0 -50 0 50 0) 1500)
				      ;;(send *ri* :wait-interpolation)
				      
		  ((= timestate 1) (progn
				     (print "1")
				     ;;(send *ri* :publish-sound :on)
				     (send *ri* :angle-vector #f(0 0 -20 0 50 0) 300)
				     ;;(send *ri* :wait-interpolation)
				     (send *dxl-armed-turtlebot* :angle-vector (send *ri* :state :potentio-vector))
				     
				     
				     ))
		  ((= timestate 2) (progn
				     ;;(send *ri* :publish-sound :on)
				     (print "2")
				     (if found-obj2
					 (progn
					   (send *dxl-armed-turtlebot* :inverse-kinematics target-cds2  :revert-if-fail t :rotation-axis :z)
					   (send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 300)
					   (send *ri* :wait-interpolation)
					   ;;(ros::duration-sleep 0.7)
					   ;;(send *ri* :publish-sound :error)
					   (print "inverse")
					   )
				       (print "no obj found")
				       )
				     ))
		  ((= timestate 3) (progn
				     (if (and found-obj found-obj2) (print "1 and 2") (if (and (not found-obj) found-obj2) (progn (print "only 2") (if (eql magnetflag 1) (print "you lose"))) (if (and (not found-obj2) found-obj) (progn (print "only 1") (if (eql magnetflag 0) (print "you lose"))) (print "noexist,error so i lose"))))
				     ;;(send *ri* :publish-sound :off)
				     (print "3")
				     (send *ri* :angle-vector #f(0 0 -20 0 50 0) 300)
				     (send *ri* :wait-interpolation)
				     (send *dxl-armed-turtlebot* :angle-vector (send *ri* :state :potentio-vector))
				     
				     
				     )))
		 (setq timechange nil)
		 )
       )
     )
   )
       (setq countsave (+ 1 countsave))
       (setq countsave2 (+ 1 countsave2))
       (if (> countsave 5)
	   (setq found-obj nil)
	 )
       (if (> countsave2 5)
	   (setq found-obj2 nil)
	 )
       
 
   (send *irtviewer* :draw-objects)
   (x::window-main-one)
   (ros::spin-once)
   (ros::sleep)
   
   )
