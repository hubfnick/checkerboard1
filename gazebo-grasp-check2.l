#!/usr/bin/env roseus

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)


(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)
(load "models/chessboard-30-7x5-object.l")
(setq found-obj nil)
(setq obj-pos #f(0 0 0))

(ros::roseus "tf-listener")
(setq timestate 0)
(setq timechange t)


(setq termstart (ros::time-now))
(do-until-key
 (setq curtime (ros::time-now))
 (if (ros::time>= (ros::time- curtime termstart) (ros::time 3.0))
     (progn
       (format t "elapsed ~A sec ~%" (send (ros::time- (ros::time-now) termstart) :to-sec))
       (setq termstart curtime)
       (setq timestate (mod (+ timestate 1) 4))
       (setq timechange t)
       )
   )
 (if timechange
     (progn
       (cond ((= timestate 0) (progn
				(print "0")
				(send *ri* :angle-vector #f(0 0 -50 0 50 0) 1500)
				(send *ri* :wait-interpolation)
				))
	     ((= timestate 1) (progn
				(print "1")
				(send *ri* :angle-vector #f(0 0 0 0 0 0) 1500)
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
 
 )
