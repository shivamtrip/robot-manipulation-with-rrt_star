; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GetDigitalIO-request.msg.html

(cl:defclass <GetDigitalIO-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDigitalIO-request (<GetDigitalIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDigitalIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDigitalIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetDigitalIO-request> is deprecated: use xarm_msgs-srv:GetDigitalIO-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDigitalIO-request>) ostream)
  "Serializes a message object of type '<GetDigitalIO-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDigitalIO-request>) istream)
  "Deserializes a message object of type '<GetDigitalIO-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDigitalIO-request>)))
  "Returns string type for a service object of type '<GetDigitalIO-request>"
  "xarm_msgs/GetDigitalIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDigitalIO-request)))
  "Returns string type for a service object of type 'GetDigitalIO-request"
  "xarm_msgs/GetDigitalIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDigitalIO-request>)))
  "Returns md5sum for a message object of type '<GetDigitalIO-request>"
  "5c2bfd923228c71a217e97a1d1747b99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDigitalIO-request)))
  "Returns md5sum for a message object of type 'GetDigitalIO-request"
  "5c2bfd923228c71a217e97a1d1747b99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDigitalIO-request>)))
  "Returns full string definition for message of type '<GetDigitalIO-request>"
  (cl:format cl:nil "# Getting the 2 digital Input port at robot end connector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDigitalIO-request)))
  "Returns full string definition for message of type 'GetDigitalIO-request"
  (cl:format cl:nil "# Getting the 2 digital Input port at robot end connector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDigitalIO-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDigitalIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDigitalIO-request
))
;//! \htmlinclude GetDigitalIO-response.msg.html

(cl:defclass <GetDigitalIO-response> (roslisp-msg-protocol:ros-message)
  ((digital_1
    :reader digital_1
    :initarg :digital_1
    :type cl:integer
    :initform 0)
   (digital_2
    :reader digital_2
    :initarg :digital_2
    :type cl:integer
    :initform 0)
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

(cl:defclass GetDigitalIO-response (<GetDigitalIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDigitalIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDigitalIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetDigitalIO-response> is deprecated: use xarm_msgs-srv:GetDigitalIO-response instead.")))

(cl:ensure-generic-function 'digital_1-val :lambda-list '(m))
(cl:defmethod digital_1-val ((m <GetDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:digital_1-val is deprecated.  Use xarm_msgs-srv:digital_1 instead.")
  (digital_1 m))

(cl:ensure-generic-function 'digital_2-val :lambda-list '(m))
(cl:defmethod digital_2-val ((m <GetDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:digital_2-val is deprecated.  Use xarm_msgs-srv:digital_2 instead.")
  (digital_2 m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GetDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetDigitalIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDigitalIO-response>) ostream)
  "Serializes a message object of type '<GetDigitalIO-response>"
  (cl:let* ((signed (cl:slot-value msg 'digital_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'digital_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDigitalIO-response>) istream)
  "Deserializes a message object of type '<GetDigitalIO-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'digital_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'digital_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDigitalIO-response>)))
  "Returns string type for a service object of type '<GetDigitalIO-response>"
  "xarm_msgs/GetDigitalIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDigitalIO-response)))
  "Returns string type for a service object of type 'GetDigitalIO-response"
  "xarm_msgs/GetDigitalIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDigitalIO-response>)))
  "Returns md5sum for a message object of type '<GetDigitalIO-response>"
  "5c2bfd923228c71a217e97a1d1747b99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDigitalIO-response)))
  "Returns md5sum for a message object of type 'GetDigitalIO-response"
  "5c2bfd923228c71a217e97a1d1747b99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDigitalIO-response>)))
  "Returns full string definition for message of type '<GetDigitalIO-response>"
  (cl:format cl:nil "~%int32 digital_1~%~%int32 digital_2~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDigitalIO-response)))
  "Returns full string definition for message of type 'GetDigitalIO-response"
  (cl:format cl:nil "~%int32 digital_1~%~%int32 digital_2~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDigitalIO-response>))
  (cl:+ 0
     4
     4
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDigitalIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDigitalIO-response
    (cl:cons ':digital_1 (digital_1 msg))
    (cl:cons ':digital_2 (digital_2 msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDigitalIO)))
  'GetDigitalIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDigitalIO)))
  'GetDigitalIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDigitalIO)))
  "Returns string type for a service object of type '<GetDigitalIO>"
  "xarm_msgs/GetDigitalIO")