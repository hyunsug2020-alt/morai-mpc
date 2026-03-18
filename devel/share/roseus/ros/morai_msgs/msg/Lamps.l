;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::Lamps)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'Lamps (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::LAMPS")
  (make-package "MORAI_MSGS::LAMPS"))

(in-package "ROS")
;;//! \htmlinclude Lamps.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::Lamps
  :super ros::object
  :slots (_header _turn_signal _emergency_signal ))

(defmethod morai_msgs::Lamps
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:turn_signal __turn_signal) 0)
    ((:emergency_signal __emergency_signal) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _turn_signal (round __turn_signal))
   (setq _emergency_signal (round __emergency_signal))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:turn_signal
   (&optional __turn_signal)
   (if __turn_signal (setq _turn_signal __turn_signal)) _turn_signal)
  (:emergency_signal
   (&optional __emergency_signal)
   (if __emergency_signal (setq _emergency_signal __emergency_signal)) _emergency_signal)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _turn_signal
    1
    ;; int8 _emergency_signal
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _turn_signal
       (write-byte _turn_signal s)
     ;; int8 _emergency_signal
       (write-byte _emergency_signal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _turn_signal
     (setq _turn_signal (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _turn_signal 127) (setq _turn_signal (- _turn_signal 256)))
   ;; int8 _emergency_signal
     (setq _emergency_signal (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _emergency_signal 127) (setq _emergency_signal (- _emergency_signal 256)))
   ;;
   self)
  )

(setf (get morai_msgs::Lamps :md5sum-) "59737a050dcd2e90af49d3ccb2c9cc88")
(setf (get morai_msgs::Lamps :datatype-) "morai_msgs/Lamps")
(setf (get morai_msgs::Lamps :definition-)
      "std_msgs/Header header

int8 turn_signal
int8 emergency_signal

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



(provide :morai_msgs/Lamps "59737a050dcd2e90af49d3ccb2c9cc88")


