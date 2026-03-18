;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::SetTrafficLight)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'SetTrafficLight (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::SETTRAFFICLIGHT")
  (make-package "MORAI_MSGS::SETTRAFFICLIGHT"))

(in-package "ROS")
;;//! \htmlinclude SetTrafficLight.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::SetTrafficLight
  :super ros::object
  :slots (_header _traffic_light_index _traffic_light_status ))

(defmethod morai_msgs::SetTrafficLight
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:traffic_light_index __traffic_light_index) "")
    ((:traffic_light_status __traffic_light_status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _traffic_light_index (string __traffic_light_index))
   (setq _traffic_light_status (round __traffic_light_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:traffic_light_index
   (&optional __traffic_light_index)
   (if __traffic_light_index (setq _traffic_light_index __traffic_light_index)) _traffic_light_index)
  (:traffic_light_status
   (&optional __traffic_light_status)
   (if __traffic_light_status (setq _traffic_light_status __traffic_light_status)) _traffic_light_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _traffic_light_index
    4 (length _traffic_light_index)
    ;; int16 _traffic_light_status
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _traffic_light_index
       (write-long (length _traffic_light_index) s) (princ _traffic_light_index s)
     ;; int16 _traffic_light_status
       (write-word _traffic_light_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _traffic_light_index
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _traffic_light_index (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _traffic_light_status
     (setq _traffic_light_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get morai_msgs::SetTrafficLight :md5sum-) "1d80dc8acfeb7a2e665a8844435f9a44")
(setf (get morai_msgs::SetTrafficLight :datatype-) "morai_msgs/SetTrafficLight")
(setf (get morai_msgs::SetTrafficLight :definition-)
      "std_msgs/Header header

string traffic_light_index
int16 traffic_light_status

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



(provide :morai_msgs/SetTrafficLight "1d80dc8acfeb7a2e665a8844435f9a44")


