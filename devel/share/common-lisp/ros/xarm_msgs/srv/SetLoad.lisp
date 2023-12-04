; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetLoad-request.msg.html

(cl:defclass <SetLoad-request> (roslisp-msg-protocol:ros-message)
  ((mass
    :reader mass
    :initarg :mass
    :type cl:float
    :initform 0.0)
   (xc
    :reader xc
    :initarg :xc
    :type cl:float
    :initform 0.0)
   (yc
    :reader yc
    :initarg :yc
    :type cl:float
    :initform 0.0)
   (zc
    :reader zc
    :initarg :zc
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetLoad-request (<SetLoad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLoad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLoad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetLoad-request> is deprecated: use xarm_msgs-srv:SetLoad-request instead.")))

(cl:ensure-generic-function 'mass-val :lambda-list '(m))
(cl:defmethod mass-val ((m <SetLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:mass-val is deprecated.  Use xarm_msgs-srv:mass instead.")
  (mass m))

(cl:ensure-generic-function 'xc-val :lambda-list '(m))
(cl:defmethod xc-val ((m <SetLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:xc-val is deprecated.  Use xarm_msgs-srv:xc instead.")
  (xc m))

(cl:ensure-generic-function 'yc-val :lambda-list '(m))
(cl:defmethod yc-val ((m <SetLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:yc-val is deprecated.  Use xarm_msgs-srv:yc instead.")
  (yc m))

(cl:ensure-generic-function 'zc-val :lambda-list '(m))
(cl:defmethod zc-val ((m <SetLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:zc-val is deprecated.  Use xarm_msgs-srv:zc instead.")
  (zc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLoad-request>) ostream)
  "Serializes a message object of type '<SetLoad-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLoad-request>) istream)
  "Deserializes a message object of type '<SetLoad-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mass) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zc) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLoad-request>)))
  "Returns string type for a service object of type '<SetLoad-request>"
  "xarm_msgs/SetLoadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLoad-request)))
  "Returns string type for a service object of type 'SetLoad-request"
  "xarm_msgs/SetLoadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLoad-request>)))
  "Returns md5sum for a message object of type '<SetLoad-request>"
  "a61cba8401ac162ae54cbe464f360d3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLoad-request)))
  "Returns md5sum for a message object of type 'SetLoad-request"
  "a61cba8401ac162ae54cbe464f360d3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLoad-request>)))
  "Returns full string definition for message of type '<SetLoad-request>"
  (cl:format cl:nil "# to set robot load parameters, based on initial Tool Frame located at flange center.~%# Load mass (kg)~%float32 mass~%~%# Load Center of Mass (mm), with respect to intial Tool Frame~%float32 xc~%float32 yc~%float32 zc~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLoad-request)))
  "Returns full string definition for message of type 'SetLoad-request"
  (cl:format cl:nil "# to set robot load parameters, based on initial Tool Frame located at flange center.~%# Load mass (kg)~%float32 mass~%~%# Load Center of Mass (mm), with respect to intial Tool Frame~%float32 xc~%float32 yc~%float32 zc~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLoad-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLoad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLoad-request
    (cl:cons ':mass (mass msg))
    (cl:cons ':xc (xc msg))
    (cl:cons ':yc (yc msg))
    (cl:cons ':zc (zc msg))
))
;//! \htmlinclude SetLoad-response.msg.html

(cl:defclass <SetLoad-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetLoad-response (<SetLoad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLoad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLoad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetLoad-response> is deprecated: use xarm_msgs-srv:SetLoad-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLoad-response>) ostream)
  "Serializes a message object of type '<SetLoad-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLoad-response>) istream)
  "Deserializes a message object of type '<SetLoad-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLoad-response>)))
  "Returns string type for a service object of type '<SetLoad-response>"
  "xarm_msgs/SetLoadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLoad-response)))
  "Returns string type for a service object of type 'SetLoad-response"
  "xarm_msgs/SetLoadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLoad-response>)))
  "Returns md5sum for a message object of type '<SetLoad-response>"
  "a61cba8401ac162ae54cbe464f360d3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLoad-response)))
  "Returns md5sum for a message object of type 'SetLoad-response"
  "a61cba8401ac162ae54cbe464f360d3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLoad-response>)))
  "Returns full string definition for message of type '<SetLoad-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLoad-response)))
  "Returns full string definition for message of type 'SetLoad-response"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLoad-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLoad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLoad-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLoad)))
  'SetLoad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLoad)))
  'SetLoad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLoad)))
  "Returns string type for a service object of type '<SetLoad>"
  "xarm_msgs/SetLoad")