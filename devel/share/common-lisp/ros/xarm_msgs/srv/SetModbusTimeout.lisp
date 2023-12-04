; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetModbusTimeout-request.msg.html

(cl:defclass <SetModbusTimeout-request> (roslisp-msg-protocol:ros-message)
  ((timeout_ms
    :reader timeout_ms
    :initarg :timeout_ms
    :type cl:integer
    :initform 0)
   (is_transparent_transmission
    :reader is_transparent_transmission
    :initarg :is_transparent_transmission
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetModbusTimeout-request (<SetModbusTimeout-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModbusTimeout-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModbusTimeout-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetModbusTimeout-request> is deprecated: use xarm_msgs-srv:SetModbusTimeout-request instead.")))

(cl:ensure-generic-function 'timeout_ms-val :lambda-list '(m))
(cl:defmethod timeout_ms-val ((m <SetModbusTimeout-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:timeout_ms-val is deprecated.  Use xarm_msgs-srv:timeout_ms instead.")
  (timeout_ms m))

(cl:ensure-generic-function 'is_transparent_transmission-val :lambda-list '(m))
(cl:defmethod is_transparent_transmission-val ((m <SetModbusTimeout-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:is_transparent_transmission-val is deprecated.  Use xarm_msgs-srv:is_transparent_transmission instead.")
  (is_transparent_transmission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModbusTimeout-request>) ostream)
  "Serializes a message object of type '<SetModbusTimeout-request>"
  (cl:let* ((signed (cl:slot-value msg 'timeout_ms)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_transparent_transmission) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModbusTimeout-request>) istream)
  "Deserializes a message object of type '<SetModbusTimeout-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeout_ms) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_transparent_transmission) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModbusTimeout-request>)))
  "Returns string type for a service object of type '<SetModbusTimeout-request>"
  "xarm_msgs/SetModbusTimeoutRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusTimeout-request)))
  "Returns string type for a service object of type 'SetModbusTimeout-request"
  "xarm_msgs/SetModbusTimeoutRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModbusTimeout-request>)))
  "Returns md5sum for a message object of type '<SetModbusTimeout-request>"
  "cd53862105e4494b607dc1eaae204c5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModbusTimeout-request)))
  "Returns md5sum for a message object of type 'SetModbusTimeout-request"
  "cd53862105e4494b607dc1eaae204c5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModbusTimeout-request>)))
  "Returns full string definition for message of type '<SetModbusTimeout-request>"
  (cl:format cl:nil "# configure the timeout parameter in modbus communication, in milliseconds~%int32 timeout_ms~%~%# whether the set timeout is the timeout of transparent transmission~%bool is_transparent_transmission~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModbusTimeout-request)))
  "Returns full string definition for message of type 'SetModbusTimeout-request"
  (cl:format cl:nil "# configure the timeout parameter in modbus communication, in milliseconds~%int32 timeout_ms~%~%# whether the set timeout is the timeout of transparent transmission~%bool is_transparent_transmission~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModbusTimeout-request>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModbusTimeout-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModbusTimeout-request
    (cl:cons ':timeout_ms (timeout_ms msg))
    (cl:cons ':is_transparent_transmission (is_transparent_transmission msg))
))
;//! \htmlinclude SetModbusTimeout-response.msg.html

(cl:defclass <SetModbusTimeout-response> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetModbusTimeout-response (<SetModbusTimeout-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModbusTimeout-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModbusTimeout-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetModbusTimeout-response> is deprecated: use xarm_msgs-srv:SetModbusTimeout-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetModbusTimeout-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetModbusTimeout-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModbusTimeout-response>) ostream)
  "Serializes a message object of type '<SetModbusTimeout-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModbusTimeout-response>) istream)
  "Deserializes a message object of type '<SetModbusTimeout-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModbusTimeout-response>)))
  "Returns string type for a service object of type '<SetModbusTimeout-response>"
  "xarm_msgs/SetModbusTimeoutResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusTimeout-response)))
  "Returns string type for a service object of type 'SetModbusTimeout-response"
  "xarm_msgs/SetModbusTimeoutResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModbusTimeout-response>)))
  "Returns md5sum for a message object of type '<SetModbusTimeout-response>"
  "cd53862105e4494b607dc1eaae204c5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModbusTimeout-response)))
  "Returns md5sum for a message object of type 'SetModbusTimeout-response"
  "cd53862105e4494b607dc1eaae204c5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModbusTimeout-response>)))
  "Returns full string definition for message of type '<SetModbusTimeout-response>"
  (cl:format cl:nil "~%string message~%int16 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModbusTimeout-response)))
  "Returns full string definition for message of type 'SetModbusTimeout-response"
  (cl:format cl:nil "~%string message~%int16 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModbusTimeout-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModbusTimeout-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModbusTimeout-response
    (cl:cons ':message (message msg))
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetModbusTimeout)))
  'SetModbusTimeout-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetModbusTimeout)))
  'SetModbusTimeout-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusTimeout)))
  "Returns string type for a service object of type '<SetModbusTimeout>"
  "xarm_msgs/SetModbusTimeout")