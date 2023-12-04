; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude MoveAxisAngle-request.msg.html

(cl:defclass <MoveAxisAngle-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mvvelo
    :reader mvvelo
    :initarg :mvvelo
    :type cl:float
    :initform 0.0)
   (mvacc
    :reader mvacc
    :initarg :mvacc
    :type cl:float
    :initform 0.0)
   (mvtime
    :reader mvtime
    :initarg :mvtime
    :type cl:float
    :initform 0.0)
   (coord
    :reader coord
    :initarg :coord
    :type cl:fixnum
    :initform 0)
   (relative
    :reader relative
    :initarg :relative
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveAxisAngle-request (<MoveAxisAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAxisAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAxisAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveAxisAngle-request> is deprecated: use xarm_msgs-srv:MoveAxisAngle-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:pose-val is deprecated.  Use xarm_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'mvvelo-val :lambda-list '(m))
(cl:defmethod mvvelo-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:mvvelo-val is deprecated.  Use xarm_msgs-srv:mvvelo instead.")
  (mvvelo m))

(cl:ensure-generic-function 'mvacc-val :lambda-list '(m))
(cl:defmethod mvacc-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:mvacc-val is deprecated.  Use xarm_msgs-srv:mvacc instead.")
  (mvacc m))

(cl:ensure-generic-function 'mvtime-val :lambda-list '(m))
(cl:defmethod mvtime-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:mvtime-val is deprecated.  Use xarm_msgs-srv:mvtime instead.")
  (mvtime m))

(cl:ensure-generic-function 'coord-val :lambda-list '(m))
(cl:defmethod coord-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:coord-val is deprecated.  Use xarm_msgs-srv:coord instead.")
  (coord m))

(cl:ensure-generic-function 'relative-val :lambda-list '(m))
(cl:defmethod relative-val ((m <MoveAxisAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:relative-val is deprecated.  Use xarm_msgs-srv:relative instead.")
  (relative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAxisAngle-request>) ostream)
  "Serializes a message object of type '<MoveAxisAngle-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pose))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mvvelo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mvacc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mvtime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'coord)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'relative)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAxisAngle-request>) istream)
  "Deserializes a message object of type '<MoveAxisAngle-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mvvelo) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mvacc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mvtime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'coord) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'relative) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAxisAngle-request>)))
  "Returns string type for a service object of type '<MoveAxisAngle-request>"
  "xarm_msgs/MoveAxisAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAxisAngle-request)))
  "Returns string type for a service object of type 'MoveAxisAngle-request"
  "xarm_msgs/MoveAxisAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAxisAngle-request>)))
  "Returns md5sum for a message object of type '<MoveAxisAngle-request>"
  "e3ce015929065df69fd9351b8d2c8407")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAxisAngle-request)))
  "Returns md5sum for a message object of type 'MoveAxisAngle-request"
  "e3ce015929065df69fd9351b8d2c8407")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAxisAngle-request>)))
  "Returns full string definition for message of type '<MoveAxisAngle-request>"
  (cl:format cl:nil "# request: command specification for axis-angle motion executions.~%# Units:~%#	Linear: mm, mm/s, and mm/s^2.~%#   Angular: radian, radian/s and radian/s^2.~%#	time: sec~%~%# pose: target coordinate. ~%#	For Cartesian target: pose dimention is 6 for (x, y, z, rx, ry, rz)~%~%float32[] pose~%~%# mvvelo: specified maximum velocity during execution. linear or angular velocity ~%~%float32 mvvelo~%~%# mvacc: specified maximum acceleration during execution. linear or angular acceleration.~%~%float32 mvacc~%~%# mvtime: currently do not have any special meaning, please just give it 0. ~%~%float32 mvtime~%~%# coord: motion coordinate system indicator, base (0) or tool(1) coordinate.~%~%int16 coord~%~%# relative: indicator of given target is relative (1) or not (0, absolute)~%~%int16 relative~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAxisAngle-request)))
  "Returns full string definition for message of type 'MoveAxisAngle-request"
  (cl:format cl:nil "# request: command specification for axis-angle motion executions.~%# Units:~%#	Linear: mm, mm/s, and mm/s^2.~%#   Angular: radian, radian/s and radian/s^2.~%#	time: sec~%~%# pose: target coordinate. ~%#	For Cartesian target: pose dimention is 6 for (x, y, z, rx, ry, rz)~%~%float32[] pose~%~%# mvvelo: specified maximum velocity during execution. linear or angular velocity ~%~%float32 mvvelo~%~%# mvacc: specified maximum acceleration during execution. linear or angular acceleration.~%~%float32 mvacc~%~%# mvtime: currently do not have any special meaning, please just give it 0. ~%~%float32 mvtime~%~%# coord: motion coordinate system indicator, base (0) or tool(1) coordinate.~%~%int16 coord~%~%# relative: indicator of given target is relative (1) or not (0, absolute)~%~%int16 relative~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAxisAngle-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAxisAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAxisAngle-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':mvvelo (mvvelo msg))
    (cl:cons ':mvacc (mvacc msg))
    (cl:cons ':mvtime (mvtime msg))
    (cl:cons ':coord (coord msg))
    (cl:cons ':relative (relative msg))
))
;//! \htmlinclude MoveAxisAngle-response.msg.html

(cl:defclass <MoveAxisAngle-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass MoveAxisAngle-response (<MoveAxisAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAxisAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAxisAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveAxisAngle-response> is deprecated: use xarm_msgs-srv:MoveAxisAngle-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <MoveAxisAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <MoveAxisAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAxisAngle-response>) ostream)
  "Serializes a message object of type '<MoveAxisAngle-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAxisAngle-response>) istream)
  "Deserializes a message object of type '<MoveAxisAngle-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAxisAngle-response>)))
  "Returns string type for a service object of type '<MoveAxisAngle-response>"
  "xarm_msgs/MoveAxisAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAxisAngle-response)))
  "Returns string type for a service object of type 'MoveAxisAngle-response"
  "xarm_msgs/MoveAxisAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAxisAngle-response>)))
  "Returns md5sum for a message object of type '<MoveAxisAngle-response>"
  "e3ce015929065df69fd9351b8d2c8407")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAxisAngle-response)))
  "Returns md5sum for a message object of type 'MoveAxisAngle-response"
  "e3ce015929065df69fd9351b8d2c8407")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAxisAngle-response>)))
  "Returns full string definition for message of type '<MoveAxisAngle-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAxisAngle-response)))
  "Returns full string definition for message of type 'MoveAxisAngle-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAxisAngle-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAxisAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAxisAngle-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveAxisAngle)))
  'MoveAxisAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveAxisAngle)))
  'MoveAxisAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAxisAngle)))
  "Returns string type for a service object of type '<MoveAxisAngle>"
  "xarm_msgs/MoveAxisAngle")