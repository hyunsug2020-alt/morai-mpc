;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::MoraiSrvResponse)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'MoraiSrvResponse (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::MORAISRVRESPONSE")
  (make-package "MORAI_MSGS::MORAISRVRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude MoraiSrvResponse.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::MoraiSrvResponse
  :super ros::object
  :slots (_header _result ))

(defmethod morai_msgs::MoraiSrvResponse
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _result __result)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get morai_msgs::MoraiSrvResponse :md5sum-) "ab940139f218dedca20062b268cb5a0f")
(setf (get morai_msgs::MoraiSrvResponse :datatype-) "morai_msgs/MoraiSrvResponse")
(setf (get morai_msgs::MoraiSrvResponse :definition-)
      "std_msgs/Header header

bool result
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



(provide :morai_msgs/MoraiSrvResponse "ab940139f218dedca20062b268cb5a0f")


