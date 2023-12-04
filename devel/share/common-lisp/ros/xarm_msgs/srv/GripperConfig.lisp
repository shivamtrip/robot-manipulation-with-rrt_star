; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GripperConfig-request.msg.html

(cl:defclass <GripperConfig-request> (roslisp-msg-protocol:ros-message)
  ((pulse_vel
    :reader pulse_vel
    :initarg :pulse_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperConfig-request (<GripperConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperConfig-request> is deprecated: use xarm_msgs-srv:GripperConfig-request instead.")))

(cl:ensure-generic-function 'pulse_vel-val :lambda-list '(m))
(cl:defmethod pulse_vel-val ((m <GripperConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:pulse_vel-val is deprecated.  Use xarm_msgs-srv:pulse_vel instead.")
  (pulse_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperConfig-request>) ostream)
  "Serializes a message object of type '<GripperConfig-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pulse_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperConfig-request>) istream)
  "Deserializes a message object of type '<GripperConfig-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pulse_vel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperConfig-request>)))
  "Returns string type for a service object of type '<GripperConfig-request>"
  "xarm_msgs/GripperConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperConfig-request)))
  "Returns string type for a service object of type 'GripperConfig-request"
  "xarm_msgs/GripperConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperConfig-request>)))
  "Returns md5sum for a message object of type '<GripperConfig-request>"
  "52f0b064a7be31185f9e28fcaa5260ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperConfig-request)))
  "Returns md5sum for a message object of type 'GripperConfig-request"
  "52f0b064a7be31185f9e28fcaa5260ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperConfig-request>)))
  "Returns full string definition for message of type '<GripperConfig-request>"
  (cl:format cl:nil "# Grip velocity configuration: range is from 1 to 5000~%float32 pulse_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperConfig-request)))
  "Returns full string definition for message of type 'GripperConfig-request"
  (cl:format cl:nil "# Grip velocity configuration: range is from 1 to 5000~%float32 pulse_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperConfig-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperConfig-request
    (cl:cons ':pulse_vel (pulse_vel msg))
))
;//! \htmlinclude GripperConfig-response.msg.html

(cl:defclass <GripperConfig-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GripperConfig-response (<GripperConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GripperConfig-response> is deprecated: use xarm_msgs-srv:GripperConfig-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GripperConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GripperConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperConfig-response>) ostream)
  "Serializes a message object of type '<GripperConfig-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperConfig-response>) istream)
  "Deserializes a message object of type '<GripperConfig-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperConfig-response>)))
  "Returns string type for a service object of type '<GripperConfig-response>"
  "xarm_msgs/GripperConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperConfig-response)))
  "Returns string type for a service object of type 'GripperConfig-response"
  "xarm_msgs/GripperConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperConfig-response>)))
  "Returns md5sum for a message object of type '<GripperConfig-response>"
  "52f0b064a7be31185f9e28fcaa5260ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperConfig-response)))
  "Returns md5sum for a message object of type 'GripperConfig-response"
  "52f0b064a7be31185f9e28fcaa5260ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperConfig-response>)))
  "Returns full string definition for message of type '<GripperConfig-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperConfig-response)))
  "Returns full string definition for message of type 'GripperConfig-response"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperConfig-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperConfig-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperConfig)))
  'GripperConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperConfig)))
  'GripperConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperConfig)))
  "Returns string type for a service object of type '<GripperConfig>"
  "xarm_msgs/GripperConfig")