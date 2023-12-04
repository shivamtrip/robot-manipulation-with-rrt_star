; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetAxis-request.msg.html

(cl:defclass <SetAxis-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetAxis-request (<SetAxis-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAxis-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAxis-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetAxis-request> is deprecated: use xarm_msgs-srv:SetAxis-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetAxis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:id-val is deprecated.  Use xarm_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetAxis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:data-val is deprecated.  Use xarm_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAxis-request>) ostream)
  "Serializes a message object of type '<SetAxis-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAxis-request>) istream)
  "Deserializes a message object of type '<SetAxis-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAxis-request>)))
  "Returns string type for a service object of type '<SetAxis-request>"
  "xarm_msgs/SetAxisRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxis-request)))
  "Returns string type for a service object of type 'SetAxis-request"
  "xarm_msgs/SetAxisRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAxis-request>)))
  "Returns md5sum for a message object of type '<SetAxis-request>"
  "ba6aa16ce6b93641ff59c1e27e5efa94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAxis-request)))
  "Returns md5sum for a message object of type 'SetAxis-request"
  "ba6aa16ce6b93641ff59c1e27e5efa94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAxis-request>)))
  "Returns full string definition for message of type '<SetAxis-request>"
  (cl:format cl:nil "# request: for enabling / disabling motion control of one(or all) joint.~%# id: 1-7 for target joint number, or 8 for all joints~%# data: 0 for disabling servo control, 1 for enabling servo control.~%~%int16 id~%int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAxis-request)))
  "Returns full string definition for message of type 'SetAxis-request"
  (cl:format cl:nil "# request: for enabling / disabling motion control of one(or all) joint.~%# id: 1-7 for target joint number, or 8 for all joints~%# data: 0 for disabling servo control, 1 for enabling servo control.~%~%int16 id~%int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAxis-request>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAxis-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAxis-request
    (cl:cons ':id (id msg))
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetAxis-response.msg.html

(cl:defclass <SetAxis-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetAxis-response (<SetAxis-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAxis-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAxis-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetAxis-response> is deprecated: use xarm_msgs-srv:SetAxis-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetAxis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetAxis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAxis-response>) ostream)
  "Serializes a message object of type '<SetAxis-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAxis-response>) istream)
  "Deserializes a message object of type '<SetAxis-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAxis-response>)))
  "Returns string type for a service object of type '<SetAxis-response>"
  "xarm_msgs/SetAxisResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxis-response)))
  "Returns string type for a service object of type 'SetAxis-response"
  "xarm_msgs/SetAxisResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAxis-response>)))
  "Returns md5sum for a message object of type '<SetAxis-response>"
  "ba6aa16ce6b93641ff59c1e27e5efa94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAxis-response)))
  "Returns md5sum for a message object of type 'SetAxis-response"
  "ba6aa16ce6b93641ff59c1e27e5efa94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAxis-response>)))
  "Returns full string definition for message of type '<SetAxis-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAxis-response)))
  "Returns full string definition for message of type 'SetAxis-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAxis-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAxis-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAxis-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAxis)))
  'SetAxis-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAxis)))
  'SetAxis-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxis)))
  "Returns string type for a service object of type '<SetAxis>"
  "xarm_msgs/SetAxis")