#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")

(ros::roseus "tf-listener")
(setq timestate 0)
(setq timechange t)

(setq termstart (ros::time-now))
(do-until-key
 (setq curtime (ros::time-now))
 (if (ros::time>= (ros::time- curtime termstart) (ros::time 5.0))
     (progn
       (format t "elapsed ~A sec ~%" (send (ros::time- (ros::time-now) termstart) :to-sec))
       (setq termstart curtime)
       (setq timestate (mod (+ timestate 1) 4))
       (setq timechange t)
       )
   )
 (if timechange(progn
		 (cond ((= timestate 0) (progn
					  (print "0")
					  
					  ))
		       ((= timestate 1) (progn
					  (print "1")
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
