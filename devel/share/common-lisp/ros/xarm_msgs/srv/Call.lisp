; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude Call-request.msg.html

(cl:defclass <Call-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Call-request (<Call-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Call-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Call-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<Call-request> is deprecated: use xarm_msgs-srv:Call-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Call-request>) ostream)
  "Serializes a message object of type '<Call-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Call-request>) istream)
  "Deserializes a message object of type '<Call-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Call-request>)))
  "Returns string type for a service object of type '<Call-request>"
  "xarm_msgs/CallRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Call-request)))
  "Returns string type for a service object of type 'Call-request"
  "xarm_msgs/CallRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Call-request>)))
  "Returns md5sum for a message object of type '<Call-request>"
  "76c68a2c5e109f4d6a4dc1cfc355f405")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Call-request)))
  "Returns md5sum for a message object of type 'Call-request"
  "76c68a2c5e109f4d6a4dc1cfc355f405")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Call-request>)))
  "Returns full string definition for message of type '<Call-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Call-request)))
  "Returns full string definition for message of type 'Call-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Call-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Call-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Call-request
))
;//! \htmlinclude Call-response.msg.html

(cl:defclass <Call-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass Call-response (<Call-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Call-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Call-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<Call-response> is deprecated: use xarm_msgs-srv:Call-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <Call-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Call-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Call-response>) ostream)
  "Serializes a message object of type '<Call-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Call-response>) istream)
  "Deserializes a message object of type '<Call-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Call-response>)))
  "Returns string type for a service object of type '<Call-response>"
  "xarm_msgs/CallResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Call-response)))
  "Returns string type for a service object of type 'Call-response"
  "xarm_msgs/CallResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Call-response>)))
  "Returns md5sum for a message object of type '<Call-response>"
  "76c68a2c5e109f4d6a4dc1cfc355f405")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Call-response)))
  "Returns md5sum for a message object of type 'Call-response"
  "76c68a2c5e109f4d6a4dc1cfc355f405")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Call-response>)))
  "Returns full string definition for message of type '<Call-response>"
  (cl:format cl:nil "~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Call-response)))
  "Returns full string definition for message of type 'Call-response"
  (cl:format cl:nil "~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Call-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Call-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Call-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Call)))
  'Call-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Call)))
  'Call-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Call)))
  "Returns string type for a service object of type '<Call>"
  "xarm_msgs/Call")