;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::VehicleSpecIndex)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'VehicleSpecIndex (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::VEHICLESPECINDEX")
  (make-package "MORAI_MSGS::VEHICLESPECINDEX"))

(in-package "ROS")
;;//! \htmlinclude VehicleSpecIndex.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::VehicleSpecIndex
  :super ros::object
  :slots (_header _unique_id ))

(defmethod morai_msgs::VehicleSpecIndex
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:unique_id __unique_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _unique_id (round __unique_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:unique_id
   (&optional __unique_id)
   (if __unique_id (setq _unique_id __unique_id)) _unique_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _unique_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _unique_id
       (write-long _unique_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _unique_id
     (setq _unique_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get morai_msgs::VehicleSpecIndex :md5sum-) "46863621f323f1962bfc6281e3e3269d")
(setf (get morai_msgs::VehicleSpecIndex :datatype-) "morai_msgs/VehicleSpecIndex")
(setf (get morai_msgs::VehicleSpecIndex :definition-)
      "std_msgs/Header header

int32 unique_id
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



(provide :morai_msgs/VehicleSpecIndex "46863621f323f1962bfc6281e3e3269d")


