; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude MoraiSrvResponse.msg.html

(cl:defclass <MoraiSrvResponse> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoraiSrvResponse (<MoraiSrvResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoraiSrvResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoraiSrvResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<MoraiSrvResponse> is deprecated: use morai_msgs-msg:MoraiSrvResponse instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MoraiSrvResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:header-val is deprecated.  Use morai_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <MoraiSrvResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:result-val is deprecated.  Use morai_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoraiSrvResponse>) ostream)
  "Serializes a message object of type '<MoraiSrvResponse>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoraiSrvResponse>) istream)
  "Deserializes a message object of type '<MoraiSrvResponse>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoraiSrvResponse>)))
  "Returns string type for a message object of type '<MoraiSrvResponse>"
  "morai_msgs/MoraiSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoraiSrvResponse)))
  "Returns string type for a message object of type 'MoraiSrvResponse"
  "morai_msgs/MoraiSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoraiSrvResponse>)))
  "Returns md5sum for a message object of type '<MoraiSrvResponse>"
  "ab940139f218dedca20062b268cb5a0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoraiSrvResponse)))
  "Returns md5sum for a message object of type 'MoraiSrvResponse"
  "ab940139f218dedca20062b268cb5a0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoraiSrvResponse>)))
  "Returns full string definition for message of type '<MoraiSrvResponse>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool result~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoraiSrvResponse)))
  "Returns full string definition for message of type 'MoraiSrvResponse"
  (cl:format cl:nil "std_msgs/Header header~%~%bool result~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoraiSrvResponse>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoraiSrvResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'MoraiSrvResponse
    (cl:cons ':header (header msg))
    (cl:cons ':result (result msg))
))
