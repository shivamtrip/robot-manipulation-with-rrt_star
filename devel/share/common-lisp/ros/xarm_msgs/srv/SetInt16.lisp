; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetInt16-request.msg.html

(cl:defclass <SetInt16-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetInt16-request (<SetInt16-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInt16-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInt16-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetInt16-request> is deprecated: use xarm_msgs-srv:SetInt16-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetInt16-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:data-val is deprecated.  Use xarm_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInt16-request>) ostream)
  "Serializes a message object of type '<SetInt16-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInt16-request>) istream)
  "Deserializes a message object of type '<SetInt16-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInt16-request>)))
  "Returns string type for a service object of type '<SetInt16-request>"
  "xarm_msgs/SetInt16Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt16-request)))
  "Returns string type for a service object of type 'SetInt16-request"
  "xarm_msgs/SetInt16Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInt16-request>)))
  "Returns md5sum for a message object of type '<SetInt16-request>"
  "b107d9652f6d76aa4c20bfd2c517f4f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInt16-request)))
  "Returns md5sum for a message object of type 'SetInt16-request"
  "b107d9652f6d76aa4c20bfd2c517f4f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInt16-request>)))
  "Returns full string definition for message of type '<SetInt16-request>"
  (cl:format cl:nil "# request: set robot state or mode.~%# data: int value indicating command robot state or execution mode.~%~%int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInt16-request)))
  "Returns full string definition for message of type 'SetInt16-request"
  (cl:format cl:nil "# request: set robot state or mode.~%# data: int value indicating command robot state or execution mode.~%~%int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInt16-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInt16-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInt16-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetInt16-response.msg.html

(cl:defclass <SetInt16-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetInt16-response (<SetInt16-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInt16-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInt16-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetInt16-response> is deprecated: use xarm_msgs-srv:SetInt16-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetInt16-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetInt16-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInt16-response>) ostream)
  "Serializes a message object of type '<SetInt16-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInt16-response>) istream)
  "Deserializes a message object of type '<SetInt16-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInt16-response>)))
  "Returns string type for a service object of type '<SetInt16-response>"
  "xarm_msgs/SetInt16Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt16-response)))
  "Returns string type for a service object of type 'SetInt16-response"
  "xarm_msgs/SetInt16Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInt16-response>)))
  "Returns md5sum for a message object of type '<SetInt16-response>"
  "b107d9652f6d76aa4c20bfd2c517f4f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInt16-response)))
  "Returns md5sum for a message object of type 'SetInt16-response"
  "b107d9652f6d76aa4c20bfd2c517f4f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInt16-response>)))
  "Returns full string definition for message of type '<SetInt16-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInt16-response)))
  "Returns full string definition for message of type 'SetInt16-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInt16-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInt16-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInt16-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInt16)))
  'SetInt16-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInt16)))
  'SetInt16-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt16)))
  "Returns string type for a service object of type '<SetInt16>"
  "xarm_msgs/SetInt16")