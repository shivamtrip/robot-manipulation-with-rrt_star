; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetControllerAnalogIO-request.msg.html

(cl:defclass <SetControllerAnalogIO-request> (roslisp-msg-protocol:ros-message)
  ((port_num
    :reader port_num
    :initarg :port_num
    :type cl:fixnum
    :initform 0)
   (analog_value
    :reader analog_value
    :initarg :analog_value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetControllerAnalogIO-request (<SetControllerAnalogIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerAnalogIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerAnalogIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetControllerAnalogIO-request> is deprecated: use xarm_msgs-srv:SetControllerAnalogIO-request instead.")))

(cl:ensure-generic-function 'port_num-val :lambda-list '(m))
(cl:defmethod port_num-val ((m <SetControllerAnalogIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:port_num-val is deprecated.  Use xarm_msgs-srv:port_num instead.")
  (port_num m))

(cl:ensure-generic-function 'analog_value-val :lambda-list '(m))
(cl:defmethod analog_value-val ((m <SetControllerAnalogIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:analog_value-val is deprecated.  Use xarm_msgs-srv:analog_value instead.")
  (analog_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerAnalogIO-request>) ostream)
  "Serializes a message object of type '<SetControllerAnalogIO-request>"
  (cl:let* ((signed (cl:slot-value msg 'port_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'analog_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerAnalogIO-request>) istream)
  "Deserializes a message object of type '<SetControllerAnalogIO-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port_num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'analog_value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerAnalogIO-request>)))
  "Returns string type for a service object of type '<SetControllerAnalogIO-request>"
  "xarm_msgs/SetControllerAnalogIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerAnalogIO-request)))
  "Returns string type for a service object of type 'SetControllerAnalogIO-request"
  "xarm_msgs/SetControllerAnalogIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerAnalogIO-request>)))
  "Returns md5sum for a message object of type '<SetControllerAnalogIO-request>"
  "2b78c87698704b2bc72cea876ba90e55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerAnalogIO-request)))
  "Returns md5sum for a message object of type 'SetControllerAnalogIO-request"
  "2b78c87698704b2bc72cea876ba90e55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerAnalogIO-request>)))
  "Returns full string definition for message of type '<SetControllerAnalogIO-request>"
  (cl:format cl:nil "# Setting the analog Output port value at robot controller, io_num: from 1 to 2~%~%int16 port_num~%~%float32 analog_value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerAnalogIO-request)))
  "Returns full string definition for message of type 'SetControllerAnalogIO-request"
  (cl:format cl:nil "# Setting the analog Output port value at robot controller, io_num: from 1 to 2~%~%int16 port_num~%~%float32 analog_value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerAnalogIO-request>))
  (cl:+ 0
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerAnalogIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerAnalogIO-request
    (cl:cons ':port_num (port_num msg))
    (cl:cons ':analog_value (analog_value msg))
))
;//! \htmlinclude SetControllerAnalogIO-response.msg.html

(cl:defclass <SetControllerAnalogIO-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetControllerAnalogIO-response (<SetControllerAnalogIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerAnalogIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerAnalogIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetControllerAnalogIO-response> is deprecated: use xarm_msgs-srv:SetControllerAnalogIO-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetControllerAnalogIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetControllerAnalogIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerAnalogIO-response>) ostream)
  "Serializes a message object of type '<SetControllerAnalogIO-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerAnalogIO-response>) istream)
  "Deserializes a message object of type '<SetControllerAnalogIO-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerAnalogIO-response>)))
  "Returns string type for a service object of type '<SetControllerAnalogIO-response>"
  "xarm_msgs/SetControllerAnalogIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerAnalogIO-response)))
  "Returns string type for a service object of type 'SetControllerAnalogIO-response"
  "xarm_msgs/SetControllerAnalogIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerAnalogIO-response>)))
  "Returns md5sum for a message object of type '<SetControllerAnalogIO-response>"
  "2b78c87698704b2bc72cea876ba90e55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerAnalogIO-response)))
  "Returns md5sum for a message object of type 'SetControllerAnalogIO-response"
  "2b78c87698704b2bc72cea876ba90e55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerAnalogIO-response>)))
  "Returns full string definition for message of type '<SetControllerAnalogIO-response>"
  (cl:format cl:nil "~%int16 ret~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerAnalogIO-response)))
  "Returns full string definition for message of type 'SetControllerAnalogIO-response"
  (cl:format cl:nil "~%int16 ret~%~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerAnalogIO-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerAnalogIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerAnalogIO-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControllerAnalogIO)))
  'SetControllerAnalogIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControllerAnalogIO)))
  'SetControllerAnalogIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerAnalogIO)))
  "Returns string type for a service object of type '<SetControllerAnalogIO>"
  "xarm_msgs/SetControllerAnalogIO")