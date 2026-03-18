;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::TrafficLightIndex)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'TrafficLightIndex (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::TRAFFICLIGHTINDEX")
  (make-package "MORAI_MSGS::TRAFFICLIGHTINDEX"))

(in-package "ROS")
;;//! \htmlinclude TrafficLightIndex.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::TrafficLightIndex
  :super ros::object
  :slots (_header _idx ))

(defmethod morai_msgs::TrafficLightIndex
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:idx __idx) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _idx (string __idx))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _idx
    4 (length _idx)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _idx
       (write-long (length _idx) s) (princ _idx s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _idx
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _idx (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get morai_msgs::TrafficLightIndex :md5sum-) "ce6b633ce00ebf1b48574d33845dbe8b")
(setf (get morai_msgs::TrafficLightIndex :datatype-) "morai_msgs/TrafficLightIndex")
(setf (get morai_msgs::TrafficLightIndex :definition-)
      "std_msgs/Header header

string idx
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :morai_msgs/TrafficLightIndex "ce6b633ce00ebf1b48574d33845dbe8b")


