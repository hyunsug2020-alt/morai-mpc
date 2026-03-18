; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude MoraiSimProcHandle.msg.html

(cl:defclass <MoraiSimProcHandle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sim_process_status
    :reader sim_process_status
    :initarg :sim_process_status
    :type cl:fixnum
    :initform 0)
   (replay_option
    :reader replay_option
    :initarg :replay_option
    :type cl:fixnum
    :initform 0)
   (rosbag_file_name
    :reader rosbag_file_name
    :initarg :rosbag_file_name
    :type cl:string
    :initform "")
   (replay_target_option
    :reader replay_target_option
    :initarg :replay_target_option
    :type cl:fixnum
    :initform 0)
   (replay_speed
    :reader replay_speed
    :initarg :replay_speed
    :type cl:integer
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:integer
    :initform 0))
)

(cl:defclass MoraiSimProcHandle (<MoraiSimProcHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoraiSimProcHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoraiSimProcHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<MoraiSimProcHandle> is deprecated: use morai_msgs-msg:MoraiSimProcHandle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:header-val is deprecated.  Use morai_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sim_process_status-val :lambda-list '(m))
(cl:defmethod sim_process_status-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:sim_process_status-val is deprecated.  Use morai_msgs-msg:sim_process_status instead.")
  (sim_process_status m))

(cl:ensure-generic-function 'replay_option-val :lambda-list '(m))
(cl:defmethod replay_option-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:replay_option-val is deprecated.  Use morai_msgs-msg:replay_option instead.")
  (replay_option m))

(cl:ensure-generic-function 'rosbag_file_name-val :lambda-list '(m))
(cl:defmethod rosbag_file_name-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:rosbag_file_name-val is deprecated.  Use morai_msgs-msg:rosbag_file_name instead.")
  (rosbag_file_name m))

(cl:ensure-generic-function 'replay_target_option-val :lambda-list '(m))
(cl:defmethod replay_target_option-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:replay_target_option-val is deprecated.  Use morai_msgs-msg:replay_target_option instead.")
  (replay_target_option m))

(cl:ensure-generic-function 'replay_speed-val :lambda-list '(m))
(cl:defmethod replay_speed-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:replay_speed-val is deprecated.  Use morai_msgs-msg:replay_speed instead.")
  (replay_speed m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <MoraiSimProcHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:start_time-val is deprecated.  Use morai_msgs-msg:start_time instead.")
  (start_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoraiSimProcHandle>) ostream)
  "Serializes a message object of type '<MoraiSimProcHandle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sim_process_status)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'replay_option)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rosbag_file_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rosbag_file_name))
  (cl:let* ((signed (cl:slot-value msg 'replay_target_option)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'replay_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'start_time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoraiSimProcHandle>) istream)
  "Deserializes a message object of type '<MoraiSimProcHandle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sim_process_status)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'replay_option) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rosbag_file_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rosbag_file_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'replay_target_option) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'replay_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoraiSimProcHandle>)))
  "Returns string type for a message object of type '<MoraiSimProcHandle>"
  "morai_msgs/MoraiSimProcHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoraiSimProcHandle)))
  "Returns string type for a message object of type 'MoraiSimProcHandle"
  "morai_msgs/MoraiSimProcHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoraiSimProcHandle>)))
  "Returns md5sum for a message object of type '<MoraiSimProcHandle>"
  "4f90a190f9ea5b4fff26f1a9a37d16eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoraiSimProcHandle)))
  "Returns md5sum for a message object of type 'MoraiSimProcHandle"
  "4f90a190f9ea5b4fff26f1a9a37d16eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoraiSimProcHandle>)))
  "Returns full string definition for message of type '<MoraiSimProcHandle>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 sim_process_status~%~%int16 replay_option~%string rosbag_file_name~%int16 replay_target_option~%~%int32 replay_speed~%int32 start_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoraiSimProcHandle)))
  "Returns full string definition for message of type 'MoraiSimProcHandle"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 sim_process_status~%~%int16 replay_option~%string rosbag_file_name~%int16 replay_target_option~%~%int32 replay_speed~%int32 start_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoraiSimProcHandle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     4 (cl:length (cl:slot-value msg 'rosbag_file_name))
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoraiSimProcHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'MoraiSimProcHandle
    (cl:cons ':header (header msg))
    (cl:cons ':sim_process_status (sim_process_status msg))
    (cl:cons ':replay_option (replay_option msg))
    (cl:cons ':rosbag_file_name (rosbag_file_name msg))
    (cl:cons ':replay_target_option (replay_target_option msg))
    (cl:cons ':replay_speed (replay_speed msg))
    (cl:cons ':start_time (start_time msg))
))
