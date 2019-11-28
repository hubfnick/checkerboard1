#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")

(ros::roseus "tf-listener")


(setq termstart (ros::time-now))
(do-until-key
 (setq curtime (ros::time-now))
 (if (ros::time>= (ros::time- curtime termstart) (ros::time 5.0))
     (progn
       (format t "elapsed ~A sec ~%" (send (ros::time- (ros::time-now) termstart) :to-sec))
       (setq termstart curtime))
   )
)
