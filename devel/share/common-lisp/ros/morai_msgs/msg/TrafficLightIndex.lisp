; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude TrafficLightIndex.msg.html

(cl:defclass <TrafficLightIndex> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (idx
    :reader idx
    :initarg :idx
    :type cl:string
    :initform ""))
)

(cl:defclass TrafficLightIndex (<TrafficLightIndex>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficLightIndex>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficLightIndex)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<TrafficLightIndex> is deprecated: use morai_msgs-msg:TrafficLightIndex instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrafficLightIndex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:header-val is deprecated.  Use morai_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'idx-val :lambda-list '(m))
(cl:defmethod idx-val ((m <TrafficLightIndex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:idx-val is deprecated.  Use morai_msgs-msg:idx instead.")
  (idx m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficLightIndex>) ostream)
  "Serializes a message object of type '<TrafficLightIndex>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'idx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'idx))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficLightIndex>) istream)
  "Deserializes a message object of type '<TrafficLightIndex>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idx) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'idx) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficLightIndex>)))
  "Returns string type for a message object of type '<TrafficLightIndex>"
  "morai_msgs/TrafficLightIndex")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficLightIndex)))
  "Returns string type for a message object of type 'TrafficLightIndex"
  "morai_msgs/TrafficLightIndex")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficLightIndex>)))
  "Returns md5sum for a message object of type '<TrafficLightIndex>"
  "ce6b633ce00ebf1b48574d33845dbe8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficLightIndex)))
  "Returns md5sum for a message object of type 'TrafficLightIndex"
  "ce6b633ce00ebf1b48574d33845dbe8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficLightIndex>)))
  "Returns full string definition for message of type '<TrafficLightIndex>"
  (cl:format cl:nil "std_msgs/Header header~%~%string idx~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficLightIndex)))
  "Returns full string definition for message of type 'TrafficLightIndex"
  (cl:format cl:nil "std_msgs/Header header~%~%string idx~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficLightIndex>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'idx))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficLightIndex>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficLightIndex
    (cl:cons ':header (header msg))
    (cl:cons ':idx (idx msg))
))
