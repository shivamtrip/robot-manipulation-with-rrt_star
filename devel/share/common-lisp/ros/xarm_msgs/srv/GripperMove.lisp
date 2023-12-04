; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GripperMove-request.msg.html

(cl:defclass <GripperMove-request> (roslisp-msg-protocol:ros-message)
  ((pulse_pos
    :reader pulse_pos
    :initarg :pulse_pos
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperMove-request (<GripperMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperMove-request> is deprecated: use xarm_msgs-srv:GripperMove-request instead.")))

(cl:ensure-generic-function 'pulse_pos-val :lambda-list '(m))
(cl:defmethod pulse_pos-val ((m <GripperMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:pulse_pos-val is deprecated.  Use xarm_msgs-srv:pulse_pos instead.")
  (pulse_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperMove-request>) ostream)
  "Serializes a message object of type '<GripperMove-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pulse_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperMove-request>) istream)
  "Deserializes a message object of type '<GripperMove-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pulse_pos) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperMove-request>)))
  "Returns string type for a service object of type '<GripperMove-request>"
  "xarm_msgs/GripperMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove-request)))
  "Returns string type for a service object of type 'GripperMove-request"
  "xarm_msgs/GripperMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperMove-request>)))
  "Returns md5sum for a message object of type '<GripperMove-request>"
  "68b32f775972a0898b388d8b66a9b5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperMove-request)))
  "Returns md5sum for a message object of type 'GripperMove-request"
  "68b32f775972a0898b388d8b66a9b5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperMove-request>)))
  "Returns full string definition for message of type '<GripperMove-request>"
  (cl:format cl:nil "# position command of gripper: range from 0(close) to 850 (open)~%float32 pulse_pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperMove-request)))
  "Returns full string definition for message of type 'GripperMove-request"
  (cl:format cl:nil "# position command of gripper: range from 0(close) to 850 (open)~%float32 pulse_pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperMove-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperMove-request
    (cl:cons ':pulse_pos (pulse_pos msg))
))
;//! \htmlinclude GripperMove-response.msg.html

(cl:defclass <GripperMove-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GripperMove-response (<GripperMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperMove-response> is deprecated: use xarm_msgs-srv:GripperMove-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GripperMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GripperMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperMove-response>) ostream)
  "Serializes a message object of type '<GripperMove-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperMove-response>) istream)
  "Deserializes a message object of type '<GripperMove-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperMove-response>)))
  "Returns string type for a service object of type '<GripperMove-response>"
  "xarm_msgs/GripperMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove-response)))
  "Returns string type for a service object of type 'GripperMove-response"
  "xarm_msgs/GripperMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperMove-response>)))
  "Returns md5sum for a message object of type '<GripperMove-response>"
  "68b32f775972a0898b388d8b66a9b5d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperMove-response)))
  "Returns md5sum for a message object of type 'GripperMove-response"
  "68b32f775972a0898b388d8b66a9b5d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperMove-response>)))
  "Returns full string definition for message of type '<GripperMove-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperMove-response)))
  "Returns full string definition for message of type 'GripperMove-response"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperMove-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperMove-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperMove)))
  'GripperMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperMove)))
  'GripperMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMove)))
  "Returns string type for a service object of type '<GripperMove>"
  "xarm_msgs/GripperMove")