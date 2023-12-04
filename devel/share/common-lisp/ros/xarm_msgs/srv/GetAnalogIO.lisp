; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GetAnalogIO-request.msg.html

(cl:defclass <GetAnalogIO-request> (roslisp-msg-protocol:ros-message)
  ((port_num
    :reader port_num
    :initarg :port_num
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetAnalogIO-request (<GetAnalogIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAnalogIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAnalogIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetAnalogIO-request> is deprecated: use xarm_msgs-srv:GetAnalogIO-request instead.")))

(cl:ensure-generic-function 'port_num-val :lambda-list '(m))
(cl:defmethod port_num-val ((m <GetAnalogIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:port_num-val is deprecated.  Use xarm_msgs-srv:port_num instead.")
  (port_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAnalogIO-request>) ostream)
  "Serializes a message object of type '<GetAnalogIO-request>"
  (cl:let* ((signed (cl:slot-value msg 'port_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAnalogIO-request>) istream)
  "Deserializes a message object of type '<GetAnalogIO-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port_num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAnalogIO-request>)))
  "Returns string type for a service object of type '<GetAnalogIO-request>"
  "xarm_msgs/GetAnalogIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnalogIO-request)))
  "Returns string type for a service object of type 'GetAnalogIO-request"
  "xarm_msgs/GetAnalogIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAnalogIO-request>)))
  "Returns md5sum for a message object of type '<GetAnalogIO-request>"
  "be8d9a2c0ed50c727cbf098654568f97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAnalogIO-request)))
  "Returns md5sum for a message object of type 'GetAnalogIO-request"
  "be8d9a2c0ed50c727cbf098654568f97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAnalogIO-request>)))
  "Returns full string definition for message of type '<GetAnalogIO-request>"
  (cl:format cl:nil "# Getting one of the 2 analog Input port at robot end connector or controller~%~%int16 port_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAnalogIO-request)))
  "Returns full string definition for message of type 'GetAnalogIO-request"
  (cl:format cl:nil "# Getting one of the 2 analog Input port at robot end connector or controller~%~%int16 port_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAnalogIO-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAnalogIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAnalogIO-request
    (cl:cons ':port_num (port_num msg))
))
;//! \htmlinclude GetAnalogIO-response.msg.html

(cl:defclass <GetAnalogIO-response> (roslisp-msg-protocol:ros-message)
  ((analog_value
    :reader analog_value
    :initarg :analog_value
    :type cl:float
    :initform 0.0)
   (ret
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

(cl:defclass GetAnalogIO-response (<GetAnalogIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAnalogIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAnalogIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetAnalogIO-response> is deprecated: use xarm_msgs-srv:GetAnalogIO-response instead.")))

(cl:ensure-generic-function 'analog_value-val :lambda-list '(m))
(cl:defmethod analog_value-val ((m <GetAnalogIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:analog_value-val is deprecated.  Use xarm_msgs-srv:analog_value instead.")
  (analog_value m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GetAnalogIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetAnalogIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAnalogIO-response>) ostream)
  "Serializes a message object of type '<GetAnalogIO-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'analog_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAnalogIO-response>) istream)
  "Deserializes a message object of type '<GetAnalogIO-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'analog_value) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAnalogIO-response>)))
  "Returns string type for a service object of type '<GetAnalogIO-response>"
  "xarm_msgs/GetAnalogIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnalogIO-response)))
  "Returns string type for a service object of type 'GetAnalogIO-response"
  "xarm_msgs/GetAnalogIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAnalogIO-response>)))
  "Returns md5sum for a message object of type '<GetAnalogIO-response>"
  "be8d9a2c0ed50c727cbf098654568f97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAnalogIO-response)))
  "Returns md5sum for a message object of type 'GetAnalogIO-response"
  "be8d9a2c0ed50c727cbf098654568f97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAnalogIO-response>)))
  "Returns full string definition for message of type '<GetAnalogIO-response>"
  (cl:format cl:nil "~%float32 analog_value~%~%int16 ret~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAnalogIO-response)))
  "Returns full string definition for message of type 'GetAnalogIO-response"
  (cl:format cl:nil "~%float32 analog_value~%~%int16 ret~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAnalogIO-response>))
  (cl:+ 0
     4
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAnalogIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAnalogIO-response
    (cl:cons ':analog_value (analog_value msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAnalogIO)))
  'GetAnalogIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAnalogIO)))
  'GetAnalogIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnalogIO)))
  "Returns string type for a service object of type '<GetAnalogIO>"
  "xarm_msgs/GetAnalogIO")