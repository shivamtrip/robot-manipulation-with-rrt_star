; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GripperState-request.msg.html

(cl:defclass <GripperState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GripperState-request (<GripperState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperState-request> is deprecated: use xarm_msgs-srv:GripperState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperState-request>) ostream)
  "Serializes a message object of type '<GripperState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperState-request>) istream)
  "Deserializes a message object of type '<GripperState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperState-request>)))
  "Returns string type for a service object of type '<GripperState-request>"
  "xarm_msgs/GripperStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState-request)))
  "Returns string type for a service object of type 'GripperState-request"
  "xarm_msgs/GripperStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperState-request>)))
  "Returns md5sum for a message object of type '<GripperState-request>"
  "b5eb0261d03e545bc9905bb8e7e041a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperState-request)))
  "Returns md5sum for a message object of type 'GripperState-request"
  "b5eb0261d03e545bc9905bb8e7e041a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperState-request>)))
  "Returns full string definition for message of type '<GripperState-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperState-request)))
  "Returns full string definition for message of type 'GripperState-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperState-request
))
;//! \htmlinclude GripperState-response.msg.html

(cl:defclass <GripperState-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (curr_pos
    :reader curr_pos
    :initarg :curr_pos
    :type cl:float
    :initform 0.0)
   (err_code
    :reader err_code
    :initarg :err_code
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GripperState-response (<GripperState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperState-response> is deprecated: use xarm_msgs-srv:GripperState-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'curr_pos-val :lambda-list '(m))
(cl:defmethod curr_pos-val ((m <GripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:curr_pos-val is deprecated.  Use xarm_msgs-srv:curr_pos instead.")
  (curr_pos m))

(cl:ensure-generic-function 'err_code-val :lambda-list '(m))
(cl:defmethod err_code-val ((m <GripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:err_code-val is deprecated.  Use xarm_msgs-srv:err_code instead.")
  (err_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperState-response>) ostream)
  "Serializes a message object of type '<GripperState-response>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'curr_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'err_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperState-response>) istream)
  "Deserializes a message object of type '<GripperState-response>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curr_pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'err_code) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperState-response>)))
  "Returns string type for a service object of type '<GripperState-response>"
  "xarm_msgs/GripperStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState-response)))
  "Returns string type for a service object of type 'GripperState-response"
  "xarm_msgs/GripperStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperState-response>)))
  "Returns md5sum for a message object of type '<GripperState-response>"
  "b5eb0261d03e545bc9905bb8e7e041a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperState-response)))
  "Returns md5sum for a message object of type 'GripperState-response"
  "b5eb0261d03e545bc9905bb8e7e041a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperState-response>)))
  "Returns full string definition for message of type '<GripperState-response>"
  (cl:format cl:nil "int16 ret~%string message~%float32 curr_pos~%int16 err_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperState-response)))
  "Returns full string definition for message of type 'GripperState-response"
  (cl:format cl:nil "int16 ret~%string message~%float32 curr_pos~%int16 err_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperState-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperState-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
    (cl:cons ':curr_pos (curr_pos msg))
    (cl:cons ':err_code (err_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperState)))
  'GripperState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperState)))
  'GripperState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState)))
  "Returns string type for a service object of type '<GripperState>"
  "xarm_msgs/GripperState")