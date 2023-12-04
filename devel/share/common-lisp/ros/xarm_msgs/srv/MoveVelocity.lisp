; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude MoveVelocity-request.msg.html

(cl:defclass <MoveVelocity-request> (roslisp-msg-protocol:ros-message)
  ((speeds
    :reader speeds
    :initarg :speeds
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (is_sync
    :reader is_sync
    :initarg :is_sync
    :type cl:boolean
    :initform cl:nil)
   (is_tool_coord
    :reader is_tool_coord
    :initarg :is_tool_coord
    :type cl:boolean
    :initform cl:nil)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveVelocity-request (<MoveVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveVelocity-request> is deprecated: use xarm_msgs-srv:MoveVelocity-request instead.")))

(cl:ensure-generic-function 'speeds-val :lambda-list '(m))
(cl:defmethod speeds-val ((m <MoveVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:speeds-val is deprecated.  Use xarm_msgs-srv:speeds instead.")
  (speeds m))

(cl:ensure-generic-function 'is_sync-val :lambda-list '(m))
(cl:defmethod is_sync-val ((m <MoveVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:is_sync-val is deprecated.  Use xarm_msgs-srv:is_sync instead.")
  (is_sync m))

(cl:ensure-generic-function 'is_tool_coord-val :lambda-list '(m))
(cl:defmethod is_tool_coord-val ((m <MoveVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:is_tool_coord-val is deprecated.  Use xarm_msgs-srv:is_tool_coord instead.")
  (is_tool_coord m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <MoveVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:duration-val is deprecated.  Use xarm_msgs-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveVelocity-request>) ostream)
  "Serializes a message object of type '<MoveVelocity-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'speeds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'speeds))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_sync) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_tool_coord) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveVelocity-request>) istream)
  "Deserializes a message object of type '<MoveVelocity-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'speeds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'speeds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'is_sync) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_tool_coord) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveVelocity-request>)))
  "Returns string type for a service object of type '<MoveVelocity-request>"
  "xarm_msgs/MoveVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelocity-request)))
  "Returns string type for a service object of type 'MoveVelocity-request"
  "xarm_msgs/MoveVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveVelocity-request>)))
  "Returns md5sum for a message object of type '<MoveVelocity-request>"
  "ba6e5e4cb25baf01712a273c753becc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveVelocity-request)))
  "Returns md5sum for a message object of type 'MoveVelocity-request"
  "ba6e5e4cb25baf01712a273c753becc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveVelocity-request>)))
  "Returns full string definition for message of type '<MoveVelocity-request>"
  (cl:format cl:nil "# request: command specification for velocity executions.~%# Units:~%#	joint space/angles: radian/s~%#	Cartesian space: mm/s, radian/s.~%~%# velocities: the velocity list of the joints/tcp~%#   For velo_move_joint_timed service: [joint1_velocity, ..., joint7_velocity]~%#   For velo_move_line_timed service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]~%float32[] speeds~%~%# is_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, true for yes, false for no.~%# avaiable for service velo_move_joint_timed~%bool is_sync~%~%# is_tool_coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(true) or not(false)~%# avaiable for service velo_move_line_timed~%bool is_tool_coord~%~%# the maximum duration of the speed, over this time will automatically set the speed to 0~%#   duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained~%#   duration == 0: always effective, will not stop automativally~%#   duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0~%# avaiable for firmware_version >= 1.8.0~%float32 duration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveVelocity-request)))
  "Returns full string definition for message of type 'MoveVelocity-request"
  (cl:format cl:nil "# request: command specification for velocity executions.~%# Units:~%#	joint space/angles: radian/s~%#	Cartesian space: mm/s, radian/s.~%~%# velocities: the velocity list of the joints/tcp~%#   For velo_move_joint_timed service: [joint1_velocity, ..., joint7_velocity]~%#   For velo_move_line_timed service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]~%float32[] speeds~%~%# is_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, true for yes, false for no.~%# avaiable for service velo_move_joint_timed~%bool is_sync~%~%# is_tool_coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(true) or not(false)~%# avaiable for service velo_move_line_timed~%bool is_tool_coord~%~%# the maximum duration of the speed, over this time will automatically set the speed to 0~%#   duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained~%#   duration == 0: always effective, will not stop automativally~%#   duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0~%# avaiable for firmware_version >= 1.8.0~%float32 duration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveVelocity-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'speeds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveVelocity-request
    (cl:cons ':speeds (speeds msg))
    (cl:cons ':is_sync (is_sync msg))
    (cl:cons ':is_tool_coord (is_tool_coord msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude MoveVelocity-response.msg.html

(cl:defclass <MoveVelocity-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass MoveVelocity-response (<MoveVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveVelocity-response> is deprecated: use xarm_msgs-srv:MoveVelocity-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <MoveVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <MoveVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveVelocity-response>) ostream)
  "Serializes a message object of type '<MoveVelocity-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveVelocity-response>) istream)
  "Deserializes a message object of type '<MoveVelocity-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveVelocity-response>)))
  "Returns string type for a service object of type '<MoveVelocity-response>"
  "xarm_msgs/MoveVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelocity-response)))
  "Returns string type for a service object of type 'MoveVelocity-response"
  "xarm_msgs/MoveVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveVelocity-response>)))
  "Returns md5sum for a message object of type '<MoveVelocity-response>"
  "ba6e5e4cb25baf01712a273c753becc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveVelocity-response)))
  "Returns md5sum for a message object of type 'MoveVelocity-response"
  "ba6e5e4cb25baf01712a273c753becc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveVelocity-response>)))
  "Returns full string definition for message of type '<MoveVelocity-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveVelocity-response)))
  "Returns full string definition for message of type 'MoveVelocity-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveVelocity-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveVelocity-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveVelocity)))
  'MoveVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveVelocity)))
  'MoveVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelocity)))
  "Returns string type for a service object of type '<MoveVelocity>"
  "xarm_msgs/MoveVelocity")