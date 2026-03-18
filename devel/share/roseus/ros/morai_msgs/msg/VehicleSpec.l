;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::VehicleSpec)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'VehicleSpec (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::VEHICLESPEC")
  (make-package "MORAI_MSGS::VEHICLESPEC"))

(in-package "ROS")
;;//! \htmlinclude VehicleSpec.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::VehicleSpec
  :super ros::object
  :slots (_header _size _wheel_base _max_steering _overhang _rear_overhang ))

(defmethod morai_msgs::VehicleSpec
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    ((:wheel_base __wheel_base) 0.0)
    ((:max_steering __max_steering) 0.0)
    ((:overhang __overhang) 0.0)
    ((:rear_overhang __rear_overhang) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _size __size)
   (setq _wheel_base (float __wheel_base))
   (setq _max_steering (float __max_steering))
   (setq _overhang (float __overhang))
   (setq _rear_overhang (float __rear_overhang))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:wheel_base
   (&optional __wheel_base)
   (if __wheel_base (setq _wheel_base __wheel_base)) _wheel_base)
  (:max_steering
   (&optional __max_steering)
   (if __max_steering (setq _max_steering __max_steering)) _max_steering)
  (:overhang
   (&optional __overhang)
   (if __overhang (setq _overhang __overhang)) _overhang)
  (:rear_overhang
   (&optional __rear_overhang)
   (if __rear_overhang (setq _rear_overhang __rear_overhang)) _rear_overhang)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ;; float32 _wheel_base
    4
    ;; float64 _max_steering
    8
    ;; float32 _overhang
    4
    ;; float32 _rear_overhang
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;; float32 _wheel_base
       (sys::poke _wheel_base (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _max_steering
       (sys::poke _max_steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _overhang
       (sys::poke _overhang (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_overhang
       (sys::poke _rear_overhang (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; float32 _wheel_base
     (setq _wheel_base (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _max_steering
     (setq _max_steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _overhang
     (setq _overhang (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_overhang
     (setq _rear_overhang (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get morai_msgs::VehicleSpec :md5sum-) "536e994f3031002b362a51a8bc1abf6e")
(setf (get morai_msgs::VehicleSpec :datatype-) "morai_msgs/VehicleSpec")
(setf (get morai_msgs::VehicleSpec :definition-)
      "std_msgs/Header header

geometry_msgs/Vector3 size
float32 wheel_base
float64 max_steering

float32 overhang 
float32 rear_overhang

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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :morai_msgs/VehicleSpec "536e994f3031002b362a51a8bc1abf6e")


