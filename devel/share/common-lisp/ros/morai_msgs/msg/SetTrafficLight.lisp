; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude SetTrafficLight.msg.html

(cl:defclass <SetTrafficLight> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (traffic_light_index
    :reader traffic_light_index
    :initarg :traffic_light_index
    :type cl:string
    :initform "")
   (traffic_light_status
    :reader traffic_light_status
    :initarg :traffic_light_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetTrafficLight (<SetTrafficLight>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTrafficLight>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTrafficLight)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<SetTrafficLight> is deprecated: use morai_msgs-msg:SetTrafficLight instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SetTrafficLight>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:header-val is deprecated.  Use morai_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'traffic_light_index-val :lambda-list '(m))
(cl:defmethod traffic_light_index-val ((m <SetTrafficLight>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:traffic_light_index-val is deprecated.  Use morai_msgs-msg:traffic_light_index instead.")
  (traffic_light_index m))

(cl:ensure-generic-function 'traffic_light_status-val :lambda-list '(m))
(cl:defmethod traffic_light_status-val ((m <SetTrafficLight>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:traffic_light_status-val is deprecated.  Use morai_msgs-msg:traffic_light_status instead.")
  (traffic_light_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTrafficLight>) ostream)
  "Serializes a message object of type '<SetTrafficLight>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'traffic_light_index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'traffic_light_index))
  (cl:let* ((signed (cl:slot-value msg 'traffic_light_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTrafficLight>) istream)
  "Deserializes a message object of type '<SetTrafficLight>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'traffic_light_index) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'traffic_light_index) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'traffic_light_status) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTrafficLight>)))
  "Returns string type for a message object of type '<SetTrafficLight>"
  "morai_msgs/SetTrafficLight")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTrafficLight)))
  "Returns string type for a message object of type 'SetTrafficLight"
  "morai_msgs/SetTrafficLight")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTrafficLight>)))
  "Returns md5sum for a message object of type '<SetTrafficLight>"
  "1d80dc8acfeb7a2e665a8844435f9a44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTrafficLight)))
  "Returns md5sum for a message object of type 'SetTrafficLight"
  "1d80dc8acfeb7a2e665a8844435f9a44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTrafficLight>)))
  "Returns full string definition for message of type '<SetTrafficLight>"
  (cl:format cl:nil "std_msgs/Header header~%~%string traffic_light_index~%int16 traffic_light_status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTrafficLight)))
  "Returns full string definition for message of type 'SetTrafficLight"
  (cl:format cl:nil "std_msgs/Header header~%~%string traffic_light_index~%int16 traffic_light_status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTrafficLight>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'traffic_light_index))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTrafficLight>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTrafficLight
    (cl:cons ':header (header msg))
    (cl:cons ':traffic_light_index (traffic_light_index msg))
    (cl:cons ':traffic_light_status (traffic_light_status msg))
))
