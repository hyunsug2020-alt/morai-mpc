;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::SaveSensorData)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'SaveSensorData (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::SAVESENSORDATA")
  (make-package "MORAI_MSGS::SAVESENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude SaveSensorData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::SaveSensorData
  :super ros::object
  :slots (_header _is_custom_file_name _custom_file_name _file_dir ))

(defmethod morai_msgs::SaveSensorData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_custom_file_name __is_custom_file_name) nil)
    ((:custom_file_name __custom_file_name) "")
    ((:file_dir __file_dir) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_custom_file_name __is_custom_file_name)
   (setq _custom_file_name (string __custom_file_name))
   (setq _file_dir (string __file_dir))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_custom_file_name
   (&optional (__is_custom_file_name :null))
   (if (not (eq __is_custom_file_name :null)) (setq _is_custom_file_name __is_custom_file_name)) _is_custom_file_name)
  (:custom_file_name
   (&optional __custom_file_name)
   (if __custom_file_name (setq _custom_file_name __custom_file_name)) _custom_file_name)
  (:file_dir
   (&optional __file_dir)
   (if __file_dir (setq _file_dir __file_dir)) _file_dir)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_custom_file_name
    1
    ;; string _custom_file_name
    4 (length _custom_file_name)
    ;; string _file_dir
    4 (length _file_dir)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_custom_file_name
       (if _is_custom_file_name (write-byte -1 s) (write-byte 0 s))
     ;; string _custom_file_name
       (write-long (length _custom_file_name) s) (princ _custom_file_name s)
     ;; string _file_dir
       (write-long (length _file_dir) s) (princ _file_dir s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_custom_file_name
     (setq _is_custom_file_name (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _custom_file_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _custom_file_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get morai_msgs::SaveSensorData :md5sum-) "2ea26c4b6cccd1ac8432c50be268450c")
(setf (get morai_msgs::SaveSensorData :datatype-) "morai_msgs/SaveSensorData")
(setf (get morai_msgs::SaveSensorData :definition-)
      "std_msgs/Header header

bool is_custom_file_name
string custom_file_name
string file_dir

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



(provide :morai_msgs/SaveSensorData "2ea26c4b6cccd1ac8432c50be268450c")


