; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude MoveVelo-request.msg.html

(cl:defclass <MoveVelo-request> (roslisp-msg-protocol:ros-message)
  ((velocities
    :reader velocities
    :initarg :velocities
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (jnt_sync
    :reader jnt_sync
    :initarg :jnt_sync
    :type cl:fixnum
    :initform 0)
   (coord
    :reader coord
    :initarg :coord
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveVelo-request (<MoveVelo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveVelo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveVelo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveVelo-request> is deprecated: use xarm_msgs-srv:MoveVelo-request instead.")))

(cl:ensure-generic-function 'velocities-val :lambda-list '(m))
(cl:defmethod velocities-val ((m <MoveVelo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:velocities-val is deprecated.  Use xarm_msgs-srv:velocities instead.")
  (velocities m))

(cl:ensure-generic-function 'jnt_sync-val :lambda-list '(m))
(cl:defmethod jnt_sync-val ((m <MoveVelo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:jnt_sync-val is deprecated.  Use xarm_msgs-srv:jnt_sync instead.")
  (jnt_sync m))

(cl:ensure-generic-function 'coord-val :lambda-list '(m))
(cl:defmethod coord-val ((m <MoveVelo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:coord-val is deprecated.  Use xarm_msgs-srv:coord instead.")
  (coord m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveVelo-request>) ostream)
  "Serializes a message object of type '<MoveVelo-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocities))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocities))
  (cl:let* ((signed (cl:slot-value msg 'jnt_sync)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'coord)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveVelo-request>) istream)
  "Deserializes a message object of type '<MoveVelo-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocities)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jnt_sync) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'coord) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveVelo-request>)))
  "Returns string type for a service object of type '<MoveVelo-request>"
  "xarm_msgs/MoveVeloRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelo-request)))
  "Returns string type for a service object of type 'MoveVelo-request"
  "xarm_msgs/MoveVeloRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveVelo-request>)))
  "Returns md5sum for a message object of type '<MoveVelo-request>"
  "7ab1cadf314da821f631e86ead48e30d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveVelo-request)))
  "Returns md5sum for a message object of type 'MoveVelo-request"
  "7ab1cadf314da821f631e86ead48e30d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveVelo-request>)))
  "Returns full string definition for message of type '<MoveVelo-request>"
  (cl:format cl:nil "# request: command specification for velocity executions.~%# Units:~%#	joint space/angles: radian/s~%#	Cartesian space: mm/s, radian/s.~%~%# velocities: the velocity list of the joints/tcp~%#   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]~%#   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]~%float32[] velocities~%~%# jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.~%int16 jnt_sync~%~%# coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)~%int16 coord~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveVelo-request)))
  "Returns full string definition for message of type 'MoveVelo-request"
  (cl:format cl:nil "# request: command specification for velocity executions.~%# Units:~%#	joint space/angles: radian/s~%#	Cartesian space: mm/s, radian/s.~%~%# velocities: the velocity list of the joints/tcp~%#   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]~%#   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]~%float32[] velocities~%~%# jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.~%int16 jnt_sync~%~%# coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)~%int16 coord~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveVelo-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveVelo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveVelo-request
    (cl:cons ':velocities (velocities msg))
    (cl:cons ':jnt_sync (jnt_sync msg))
    (cl:cons ':coord (coord msg))
))
;//! \htmlinclude MoveVelo-response.msg.html

(cl:defclass <MoveVelo-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass MoveVelo-response (<MoveVelo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveVelo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveVelo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<MoveVelo-response> is deprecated: use xarm_msgs-srv:MoveVelo-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <MoveVelo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <MoveVelo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveVelo-response>) ostream)
  "Serializes a message object of type '<MoveVelo-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveVelo-response>) istream)
  "Deserializes a message object of type '<MoveVelo-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveVelo-response>)))
  "Returns string type for a service object of type '<MoveVelo-response>"
  "xarm_msgs/MoveVeloResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelo-response)))
  "Returns string type for a service object of type 'MoveVelo-response"
  "xarm_msgs/MoveVeloResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveVelo-response>)))
  "Returns md5sum for a message object of type '<MoveVelo-response>"
  "7ab1cadf314da821f631e86ead48e30d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveVelo-response)))
  "Returns md5sum for a message object of type 'MoveVelo-response"
  "7ab1cadf314da821f631e86ead48e30d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveVelo-response>)))
  "Returns full string definition for message of type '<MoveVelo-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveVelo-response)))
  "Returns full string definition for message of type 'MoveVelo-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveVelo-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveVelo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveVelo-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveVelo)))
  'MoveVelo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveVelo)))
  'MoveVelo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveVelo)))
  "Returns string type for a service object of type '<MoveVelo>"
  "xarm_msgs/MoveVelo")