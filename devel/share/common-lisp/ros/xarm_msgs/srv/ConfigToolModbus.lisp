; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude ConfigToolModbus-request.msg.html

(cl:defclass <ConfigToolModbus-request> (roslisp-msg-protocol:ros-message)
  ((baud_rate
    :reader baud_rate
    :initarg :baud_rate
    :type cl:integer
    :initform 0)
   (timeout_ms
    :reader timeout_ms
    :initarg :timeout_ms
    :type cl:integer
    :initform 0))
)

(cl:defclass ConfigToolModbus-request (<ConfigToolModbus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigToolModbus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigToolModbus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<ConfigToolModbus-request> is deprecated: use xarm_msgs-srv:ConfigToolModbus-request instead.")))

(cl:ensure-generic-function 'baud_rate-val :lambda-list '(m))
(cl:defmethod baud_rate-val ((m <ConfigToolModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:baud_rate-val is deprecated.  Use xarm_msgs-srv:baud_rate instead.")
  (baud_rate m))

(cl:ensure-generic-function 'timeout_ms-val :lambda-list '(m))
(cl:defmethod timeout_ms-val ((m <ConfigToolModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:timeout_ms-val is deprecated.  Use xarm_msgs-srv:timeout_ms instead.")
  (timeout_ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigToolModbus-request>) ostream)
  "Serializes a message object of type '<ConfigToolModbus-request>"
  (cl:let* ((signed (cl:slot-value msg 'baud_rate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'timeout_ms)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigToolModbus-request>) istream)
  "Deserializes a message object of type '<ConfigToolModbus-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'baud_rate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeout_ms) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigToolModbus-request>)))
  "Returns string type for a service object of type '<ConfigToolModbus-request>"
  "xarm_msgs/ConfigToolModbusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigToolModbus-request)))
  "Returns string type for a service object of type 'ConfigToolModbus-request"
  "xarm_msgs/ConfigToolModbusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigToolModbus-request>)))
  "Returns md5sum for a message object of type '<ConfigToolModbus-request>"
  "4641743544bd81148c56355ba50062be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigToolModbus-request)))
  "Returns md5sum for a message object of type 'ConfigToolModbus-request"
  "4641743544bd81148c56355ba50062be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigToolModbus-request>)))
  "Returns full string definition for message of type '<ConfigToolModbus-request>"
  (cl:format cl:nil "# configure the tool modbus communication baud rate, in bps:~%int32 baud_rate~%~%# configure the timeout parameter in modbus communication, in milliseconds~%int32 timeout_ms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigToolModbus-request)))
  "Returns full string definition for message of type 'ConfigToolModbus-request"
  (cl:format cl:nil "# configure the tool modbus communication baud rate, in bps:~%int32 baud_rate~%~%# configure the timeout parameter in modbus communication, in milliseconds~%int32 timeout_ms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigToolModbus-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigToolModbus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigToolModbus-request
    (cl:cons ':baud_rate (baud_rate msg))
    (cl:cons ':timeout_ms (timeout_ms msg))
))
;//! \htmlinclude ConfigToolModbus-response.msg.html

(cl:defclass <ConfigToolModbus-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ConfigToolModbus-response (<ConfigToolModbus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigToolModbus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigToolModbus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<ConfigToolModbus-response> is deprecated: use xarm_msgs-srv:ConfigToolModbus-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ConfigToolModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <ConfigToolModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigToolModbus-response>) ostream)
  "Serializes a message object of type '<ConfigToolModbus-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigToolModbus-response>) istream)
  "Deserializes a message object of type '<ConfigToolModbus-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigToolModbus-response>)))
  "Returns string type for a service object of type '<ConfigToolModbus-response>"
  "xarm_msgs/ConfigToolModbusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigToolModbus-response)))
  "Returns string type for a service object of type 'ConfigToolModbus-response"
  "xarm_msgs/ConfigToolModbusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigToolModbus-response>)))
  "Returns md5sum for a message object of type '<ConfigToolModbus-response>"
  "4641743544bd81148c56355ba50062be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigToolModbus-response)))
  "Returns md5sum for a message object of type 'ConfigToolModbus-response"
  "4641743544bd81148c56355ba50062be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigToolModbus-response>)))
  "Returns full string definition for message of type '<ConfigToolModbus-response>"
  (cl:format cl:nil "~%string message~%int16 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigToolModbus-response)))
  "Returns full string definition for message of type 'ConfigToolModbus-response"
  (cl:format cl:nil "~%string message~%int16 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigToolModbus-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigToolModbus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigToolModbus-response
    (cl:cons ':message (message msg))
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigToolModbus)))
  'ConfigToolModbus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigToolModbus)))
  'ConfigToolModbus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigToolModbus)))
  "Returns string type for a service object of type '<ConfigToolModbus>"
  "xarm_msgs/ConfigToolModbus")