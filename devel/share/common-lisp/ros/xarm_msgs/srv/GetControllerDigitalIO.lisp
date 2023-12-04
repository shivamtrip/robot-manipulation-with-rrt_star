; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GetControllerDigitalIO-request.msg.html

(cl:defclass <GetControllerDigitalIO-request> (roslisp-msg-protocol:ros-message)
  ((io_num
    :reader io_num
    :initarg :io_num
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetControllerDigitalIO-request (<GetControllerDigitalIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerDigitalIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerDigitalIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetControllerDigitalIO-request> is deprecated: use xarm_msgs-srv:GetControllerDigitalIO-request instead.")))

(cl:ensure-generic-function 'io_num-val :lambda-list '(m))
(cl:defmethod io_num-val ((m <GetControllerDigitalIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:io_num-val is deprecated.  Use xarm_msgs-srv:io_num instead.")
  (io_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerDigitalIO-request>) ostream)
  "Serializes a message object of type '<GetControllerDigitalIO-request>"
  (cl:let* ((signed (cl:slot-value msg 'io_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerDigitalIO-request>) istream)
  "Deserializes a message object of type '<GetControllerDigitalIO-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'io_num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerDigitalIO-request>)))
  "Returns string type for a service object of type '<GetControllerDigitalIO-request>"
  "xarm_msgs/GetControllerDigitalIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerDigitalIO-request)))
  "Returns string type for a service object of type 'GetControllerDigitalIO-request"
  "xarm_msgs/GetControllerDigitalIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerDigitalIO-request>)))
  "Returns md5sum for a message object of type '<GetControllerDigitalIO-request>"
  "a10e26b4b2c853a8b561f9587bff33b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerDigitalIO-request)))
  "Returns md5sum for a message object of type 'GetControllerDigitalIO-request"
  "a10e26b4b2c853a8b561f9587bff33b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerDigitalIO-request>)))
  "Returns full string definition for message of type '<GetControllerDigitalIO-request>"
  (cl:format cl:nil "# Getting the controller DIGITAL input port status, io_num: from 1 to 16~%~%int16 io_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerDigitalIO-request)))
  "Returns full string definition for message of type 'GetControllerDigitalIO-request"
  (cl:format cl:nil "# Getting the controller DIGITAL input port status, io_num: from 1 to 16~%~%int16 io_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerDigitalIO-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerDigitalIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerDigitalIO-request
    (cl:cons ':io_num (io_num msg))
))
;//! \htmlinclude GetControllerDigitalIO-response.msg.html

(cl:defclass <GetControllerDigitalIO-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass GetControllerDigitalIO-response (<GetControllerDigitalIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerDigitalIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerDigitalIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetControllerDigitalIO-response> is deprecated: use xarm_msgs-srv:GetControllerDigitalIO-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GetControllerDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetControllerDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:value-val is deprecated.  Use xarm_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetControllerDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerDigitalIO-response>) ostream)
  "Serializes a message object of type '<GetControllerDigitalIO-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerDigitalIO-response>) istream)
  "Deserializes a message object of type '<GetControllerDigitalIO-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerDigitalIO-response>)))
  "Returns string type for a service object of type '<GetControllerDigitalIO-response>"
  "xarm_msgs/GetControllerDigitalIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerDigitalIO-response)))
  "Returns string type for a service object of type 'GetControllerDigitalIO-response"
  "xarm_msgs/GetControllerDigitalIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerDigitalIO-response>)))
  "Returns md5sum for a message object of type '<GetControllerDigitalIO-response>"
  "a10e26b4b2c853a8b561f9587bff33b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerDigitalIO-response)))
  "Returns md5sum for a message object of type 'GetControllerDigitalIO-response"
  "a10e26b4b2c853a8b561f9587bff33b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerDigitalIO-response>)))
  "Returns full string definition for message of type '<GetControllerDigitalIO-response>"
  (cl:format cl:nil "~%int16 ret~%~%int16 value~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerDigitalIO-response)))
  "Returns full string definition for message of type 'GetControllerDigitalIO-response"
  (cl:format cl:nil "~%int16 ret~%~%int16 value~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerDigitalIO-response>))
  (cl:+ 0
     2
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerDigitalIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerDigitalIO-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':value (value msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControllerDigitalIO)))
  'GetControllerDigitalIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControllerDigitalIO)))
  'GetControllerDigitalIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerDigitalIO)))
  "Returns string type for a service object of type '<GetControllerDigitalIO>"
  "xarm_msgs/GetControllerDigitalIO")