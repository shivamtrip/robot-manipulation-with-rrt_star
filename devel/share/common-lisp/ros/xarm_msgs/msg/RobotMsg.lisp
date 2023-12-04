; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-msg)


;//! \htmlinclude RobotMsg.msg.html

(cl:defclass <RobotMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (cmdnum
    :reader cmdnum
    :initarg :cmdnum
    :type cl:fixnum
    :initform 0)
   (mt_brake
    :reader mt_brake
    :initarg :mt_brake
    :type cl:fixnum
    :initform 0)
   (mt_able
    :reader mt_able
    :initarg :mt_able
    :type cl:fixnum
    :initform 0)
   (err
    :reader err
    :initarg :err
    :type cl:fixnum
    :initform 0)
   (warn
    :reader warn
    :initarg :warn
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (pose
    :reader pose
    :initarg :pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (offset
    :reader offset
    :initarg :offset
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RobotMsg (<RobotMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-msg:<RobotMsg> is deprecated: use xarm_msgs-msg:RobotMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:header-val is deprecated.  Use xarm_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:state-val is deprecated.  Use xarm_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:mode-val is deprecated.  Use xarm_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'cmdnum-val :lambda-list '(m))
(cl:defmethod cmdnum-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:cmdnum-val is deprecated.  Use xarm_msgs-msg:cmdnum instead.")
  (cmdnum m))

(cl:ensure-generic-function 'mt_brake-val :lambda-list '(m))
(cl:defmethod mt_brake-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:mt_brake-val is deprecated.  Use xarm_msgs-msg:mt_brake instead.")
  (mt_brake m))

(cl:ensure-generic-function 'mt_able-val :lambda-list '(m))
(cl:defmethod mt_able-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:mt_able-val is deprecated.  Use xarm_msgs-msg:mt_able instead.")
  (mt_able m))

(cl:ensure-generic-function 'err-val :lambda-list '(m))
(cl:defmethod err-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:err-val is deprecated.  Use xarm_msgs-msg:err instead.")
  (err m))

(cl:ensure-generic-function 'warn-val :lambda-list '(m))
(cl:defmethod warn-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:warn-val is deprecated.  Use xarm_msgs-msg:warn instead.")
  (warn m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:angle-val is deprecated.  Use xarm_msgs-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:pose-val is deprecated.  Use xarm_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <RobotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:offset-val is deprecated.  Use xarm_msgs-msg:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMsg>) ostream)
  "Serializes a message object of type '<RobotMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmdnum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mt_brake)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mt_able)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'err)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'warn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pose))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'offset))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMsg>) istream)
  "Deserializes a message object of type '<RobotMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmdnum) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mt_brake) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mt_able) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'err) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'warn) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angle)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:setf (cl:slot-value msg 'pose) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pose)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'offset) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'offset)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMsg>)))
  "Returns string type for a message object of type '<RobotMsg>"
  "xarm_msgs/RobotMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMsg)))
  "Returns string type for a message object of type 'RobotMsg"
  "xarm_msgs/RobotMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMsg>)))
  "Returns md5sum for a message object of type '<RobotMsg>"
  "b796af25d2a8912964fb162d2141ac39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMsg)))
  "Returns md5sum for a message object of type 'RobotMsg"
  "b796af25d2a8912964fb162d2141ac39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMsg>)))
  "Returns full string definition for message of type '<RobotMsg>"
  (cl:format cl:nil "# feedback information of the controlled robot~%Header header~%~%# state of robot:~%#	1: RUNNING, executing motion command.~%#	2: SLEEPING, not in execution, but ready to move.~%#	3: PAUSED, paused in the middle of unfinished motion.~%#	4: STOPPED, not ready for any motion commands.~%#	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.~%int16 state~%~%# mode of robot:~%#	0 for POSITION mode.(position control by xarm controller box, execute api standard commands)~%#	1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)~%#	2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)~%int16 mode~%~%# cmdnum: number of commands waiting in the buffer.~%int16 cmdnum~%~%# mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled~%int16 mt_brake~%~%# mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled~%int16 mt_able~%~%# error code (if non-zero)~%int16 err~%~%# warning code (if non-zero)~%int16 warn~%~%# current joint angles expressed in radian.~%float32[] angle~%~%# current TCP Cartesian position expressed in mm(position), radian(orientation)  ~%float32[6] pose~%~%# TCP offset from center of flange, with respect to tool frame.~%float32[6] offset~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMsg)))
  "Returns full string definition for message of type 'RobotMsg"
  (cl:format cl:nil "# feedback information of the controlled robot~%Header header~%~%# state of robot:~%#	1: RUNNING, executing motion command.~%#	2: SLEEPING, not in execution, but ready to move.~%#	3: PAUSED, paused in the middle of unfinished motion.~%#	4: STOPPED, not ready for any motion commands.~%#	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.~%int16 state~%~%# mode of robot:~%#	0 for POSITION mode.(position control by xarm controller box, execute api standard commands)~%#	1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)~%#	2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)~%int16 mode~%~%# cmdnum: number of commands waiting in the buffer.~%int16 cmdnum~%~%# mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled~%int16 mt_brake~%~%# mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled~%int16 mt_able~%~%# error code (if non-zero)~%int16 err~%~%# warning code (if non-zero)~%int16 warn~%~%# current joint angles expressed in radian.~%float32[] angle~%~%# current TCP Cartesian position expressed in mm(position), radian(orientation)  ~%float32[6] pose~%~%# TCP offset from center of flange, with respect to tool frame.~%float32[6] offset~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     2
     2
     2
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'offset) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMsg
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':cmdnum (cmdnum msg))
    (cl:cons ':mt_brake (mt_brake msg))
    (cl:cons ':mt_able (mt_able msg))
    (cl:cons ':err (err msg))
    (cl:cons ':warn (warn msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':offset (offset msg))
))
