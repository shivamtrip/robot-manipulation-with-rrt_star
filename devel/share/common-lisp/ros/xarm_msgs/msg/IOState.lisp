; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-msg)


;//! \htmlinclude IOState.msg.html

(cl:defclass <IOState> (roslisp-msg-protocol:ros-message)
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
   (analog_1
    :reader analog_1
    :initarg :analog_1
    :type cl:float
    :initform 0.0)
   (analog_2
    :reader analog_2
    :initarg :analog_2
    :type cl:float
    :initform 0.0))
)

(cl:defclass IOState (<IOState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IOState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IOState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-msg:<IOState> is deprecated: use xarm_msgs-msg:IOState instead.")))

(cl:ensure-generic-function 'digital_1-val :lambda-list '(m))
(cl:defmethod digital_1-val ((m <IOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:digital_1-val is deprecated.  Use xarm_msgs-msg:digital_1 instead.")
  (digital_1 m))

(cl:ensure-generic-function 'digital_2-val :lambda-list '(m))
(cl:defmethod digital_2-val ((m <IOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:digital_2-val is deprecated.  Use xarm_msgs-msg:digital_2 instead.")
  (digital_2 m))

(cl:ensure-generic-function 'analog_1-val :lambda-list '(m))
(cl:defmethod analog_1-val ((m <IOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:analog_1-val is deprecated.  Use xarm_msgs-msg:analog_1 instead.")
  (analog_1 m))

(cl:ensure-generic-function 'analog_2-val :lambda-list '(m))
(cl:defmethod analog_2-val ((m <IOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:analog_2-val is deprecated.  Use xarm_msgs-msg:analog_2 instead.")
  (analog_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IOState>) ostream)
  "Serializes a message object of type '<IOState>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'analog_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'analog_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IOState>) istream)
  "Deserializes a message object of type '<IOState>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'analog_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'analog_2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IOState>)))
  "Returns string type for a message object of type '<IOState>"
  "xarm_msgs/IOState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IOState)))
  "Returns string type for a message object of type 'IOState"
  "xarm_msgs/IOState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IOState>)))
  "Returns md5sum for a message object of type '<IOState>"
  "cf35d5117b3f1964a89d007e947ecbee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IOState)))
  "Returns md5sum for a message object of type 'IOState"
  "cf35d5117b3f1964a89d007e947ecbee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IOState>)))
  "Returns full string definition for message of type '<IOState>"
  (cl:format cl:nil "# for indicating 2 digital and 2 analog Input port state~%~%int32 digital_1~%~%int32 digital_2~%~%float32 analog_1~%~%float32 analog_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IOState)))
  "Returns full string definition for message of type 'IOState"
  (cl:format cl:nil "# for indicating 2 digital and 2 analog Input port state~%~%int32 digital_1~%~%int32 digital_2~%~%float32 analog_1~%~%float32 analog_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IOState>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IOState>))
  "Converts a ROS message object to a list"
  (cl:list 'IOState
    (cl:cons ':digital_1 (digital_1 msg))
    (cl:cons ':digital_2 (digital_2 msg))
    (cl:cons ':analog_1 (analog_1 msg))
    (cl:cons ':analog_2 (analog_2 msg))
))
