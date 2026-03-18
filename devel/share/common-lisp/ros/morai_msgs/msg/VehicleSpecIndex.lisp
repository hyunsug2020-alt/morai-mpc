; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude VehicleSpecIndex.msg.html

(cl:defclass <VehicleSpecIndex> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (unique_id
    :reader unique_id
    :initarg :unique_id
    :type cl:integer
    :initform 0))
)

(cl:defclass VehicleSpecIndex (<VehicleSpecIndex>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleSpecIndex>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleSpecIndex)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<VehicleSpecIndex> is deprecated: use morai_msgs-msg:VehicleSpecIndex instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehicleSpecIndex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:header-val is deprecated.  Use morai_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'unique_id-val :lambda-list '(m))
(cl:defmethod unique_id-val ((m <VehicleSpecIndex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:unique_id-val is deprecated.  Use morai_msgs-msg:unique_id instead.")
  (unique_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleSpecIndex>) ostream)
  "Serializes a message object of type '<VehicleSpecIndex>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'unique_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleSpecIndex>) istream)
  "Deserializes a message object of type '<VehicleSpecIndex>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'unique_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleSpecIndex>)))
  "Returns string type for a message object of type '<VehicleSpecIndex>"
  "morai_msgs/VehicleSpecIndex")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleSpecIndex)))
  "Returns string type for a message object of type 'VehicleSpecIndex"
  "morai_msgs/VehicleSpecIndex")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleSpecIndex>)))
  "Returns md5sum for a message object of type '<VehicleSpecIndex>"
  "46863621f323f1962bfc6281e3e3269d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleSpecIndex)))
  "Returns md5sum for a message object of type 'VehicleSpecIndex"
  "46863621f323f1962bfc6281e3e3269d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleSpecIndex>)))
  "Returns full string definition for message of type '<VehicleSpecIndex>"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 unique_id~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleSpecIndex)))
  "Returns full string definition for message of type 'VehicleSpecIndex"
  (cl:format cl:nil "std_msgs/Header header~%~%int32 unique_id~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleSpecIndex>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleSpecIndex>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleSpecIndex
    (cl:cons ':header (header msg))
    (cl:cons ':unique_id (unique_id msg))
))
