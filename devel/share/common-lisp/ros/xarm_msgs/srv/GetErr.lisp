; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GetErr-request.msg.html

(cl:defclass <GetErr-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetErr-request (<GetErr-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetErr-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetErr-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetErr-request> is deprecated: use xarm_msgs-srv:GetErr-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetErr-request>) ostream)
  "Serializes a message object of type '<GetErr-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetErr-request>) istream)
  "Deserializes a message object of type '<GetErr-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetErr-request>)))
  "Returns string type for a service object of type '<GetErr-request>"
  "xarm_msgs/GetErrRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetErr-request)))
  "Returns string type for a service object of type 'GetErr-request"
  "xarm_msgs/GetErrRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetErr-request>)))
  "Returns md5sum for a message object of type '<GetErr-request>"
  "4d513c3bc98cbaf164b4a46b30ca31f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetErr-request)))
  "Returns md5sum for a message object of type 'GetErr-request"
  "4d513c3bc98cbaf164b4a46b30ca31f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetErr-request>)))
  "Returns full string definition for message of type '<GetErr-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetErr-request)))
  "Returns full string definition for message of type 'GetErr-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetErr-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetErr-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetErr-request
))
;//! \htmlinclude GetErr-response.msg.html

(cl:defclass <GetErr-response> (roslisp-msg-protocol:ros-message)
  ((err
    :reader err
    :initarg :err
    :type cl:fixnum
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

(cl:defclass GetErr-response (<GetErr-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetErr-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetErr-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetErr-response> is deprecated: use xarm_msgs-srv:GetErr-response instead.")))

(cl:ensure-generic-function 'err-val :lambda-list '(m))
(cl:defmethod err-val ((m <GetErr-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:err-val is deprecated.  Use xarm_msgs-srv:err instead.")
  (err m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GetErr-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetErr-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetErr-response>) ostream)
  "Serializes a message object of type '<GetErr-response>"
  (cl:let* ((signed (cl:slot-value msg 'err)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetErr-response>) istream)
  "Deserializes a message object of type '<GetErr-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'err) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetErr-response>)))
  "Returns string type for a service object of type '<GetErr-response>"
  "xarm_msgs/GetErrResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetErr-response)))
  "Returns string type for a service object of type 'GetErr-response"
  "xarm_msgs/GetErrResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetErr-response>)))
  "Returns md5sum for a message object of type '<GetErr-response>"
  "4d513c3bc98cbaf164b4a46b30ca31f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetErr-response)))
  "Returns md5sum for a message object of type 'GetErr-response"
  "4d513c3bc98cbaf164b4a46b30ca31f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetErr-response>)))
  "Returns full string definition for message of type '<GetErr-response>"
  (cl:format cl:nil "~%int16 err~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetErr-response)))
  "Returns full string definition for message of type 'GetErr-response"
  (cl:format cl:nil "~%int16 err~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetErr-response>))
  (cl:+ 0
     2
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetErr-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetErr-response
    (cl:cons ':err (err msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetErr)))
  'GetErr-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetErr)))
  'GetErr-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetErr)))
  "Returns string type for a service object of type '<GetErr>"
  "xarm_msgs/GetErr")