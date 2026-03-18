;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::ScenarioLoad)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'ScenarioLoad (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::SCENARIOLOAD")
  (make-package "MORAI_MSGS::SCENARIOLOAD"))

(in-package "ROS")
;;//! \htmlinclude ScenarioLoad.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass morai_msgs::ScenarioLoad
  :super ros::object
  :slots (_header _file_name _delete_all _load_network_connection_data _load_ego_vehicle_data _load_surrounding_vehicle_data _load_pedestrian_data _load_obstacle_data _set_pause ))

(defmethod morai_msgs::ScenarioLoad
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:file_name __file_name) "")
    ((:delete_all __delete_all) nil)
    ((:load_network_connection_data __load_network_connection_data) nil)
    ((:load_ego_vehicle_data __load_ego_vehicle_data) nil)
    ((:load_surrounding_vehicle_data __load_surrounding_vehicle_data) nil)
    ((:load_pedestrian_data __load_pedestrian_data) nil)
    ((:load_obstacle_data __load_obstacle_data) nil)
    ((:set_pause __set_pause) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _file_name (string __file_name))
   (setq _delete_all __delete_all)
   (setq _load_network_connection_data __load_network_connection_data)
   (setq _load_ego_vehicle_data __load_ego_vehicle_data)
   (setq _load_surrounding_vehicle_data __load_surrounding_vehicle_data)
   (setq _load_pedestrian_data __load_pedestrian_data)
   (setq _load_obstacle_data __load_obstacle_data)
   (setq _set_pause __set_pause)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:file_name
   (&optional __file_name)
   (if __file_name (setq _file_name __file_name)) _file_name)
  (:delete_all
   (&optional (__delete_all :null))
   (if (not (eq __delete_all :null)) (setq _delete_all __delete_all)) _delete_all)
  (:load_network_connection_data
   (&optional (__load_network_connection_data :null))
   (if (not (eq __load_network_connection_data :null)) (setq _load_network_connection_data __load_network_connection_data)) _load_network_connection_data)
  (:load_ego_vehicle_data
   (&optional (__load_ego_vehicle_data :null))
   (if (not (eq __load_ego_vehicle_data :null)) (setq _load_ego_vehicle_data __load_ego_vehicle_data)) _load_ego_vehicle_data)
  (:load_surrounding_vehicle_data
   (&optional (__load_surrounding_vehicle_data :null))
   (if (not (eq __load_surrounding_vehicle_data :null)) (setq _load_surrounding_vehicle_data __load_surrounding_vehicle_data)) _load_surrounding_vehicle_data)
  (:load_pedestrian_data
   (&optional (__load_pedestrian_data :null))
   (if (not (eq __load_pedestrian_data :null)) (setq _load_pedestrian_data __load_pedestrian_data)) _load_pedestrian_data)
  (:load_obstacle_data
   (&optional (__load_obstacle_data :null))
   (if (not (eq __load_obstacle_data :null)) (setq _load_obstacle_data __load_obstacle_data)) _load_obstacle_data)
  (:set_pause
   (&optional (__set_pause :null))
   (if (not (eq __set_pause :null)) (setq _set_pause __set_pause)) _set_pause)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _file_name
    4 (length _file_name)
    ;; bool _delete_all
    1
    ;; bool _load_network_connection_data
    1
    ;; bool _load_ego_vehicle_data
    1
    ;; bool _load_surrounding_vehicle_data
    1
    ;; bool _load_pedestrian_data
    1
    ;; bool _load_obstacle_data
    1
    ;; bool _set_pause
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _file_name
       (write-long (length _file_name) s) (princ _file_name s)
     ;; bool _delete_all
       (if _delete_all (write-byte -1 s) (write-byte 0 s))
     ;; bool _load_network_connection_data
       (if _load_network_connection_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _load_ego_vehicle_data
       (if _load_ego_vehicle_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _load_surrounding_vehicle_data
       (if _load_surrounding_vehicle_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _load_pedestrian_data
       (if _load_pedestrian_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _load_obstacle_data
       (if _load_obstacle_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _set_pause
       (if _set_pause (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _file_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _delete_all
     (setq _delete_all (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _load_network_connection_data
     (setq _load_network_connection_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _load_ego_vehicle_data
     (setq _load_ego_vehicle_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _load_surrounding_vehicle_data
     (setq _load_surrounding_vehicle_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _load_pedestrian_data
     (setq _load_pedestrian_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _load_obstacle_data
     (setq _load_obstacle_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _set_pause
     (setq _set_pause (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get morai_msgs::ScenarioLoad :md5sum-) "946c9575c5f571a236ffb2bd542541cc")
(setf (get morai_msgs::ScenarioLoad :datatype-) "morai_msgs/ScenarioLoad")
(setf (get morai_msgs::ScenarioLoad :definition-)
      "std_msgs/Header header

string file_name
bool delete_all
bool load_network_connection_data
bool load_ego_vehicle_data
bool load_surrounding_vehicle_data
bool load_pedestrian_data
bool load_obstacle_data
bool set_pause

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



(provide :morai_msgs/ScenarioLoad "946c9575c5f571a236ffb2bd542541cc")


