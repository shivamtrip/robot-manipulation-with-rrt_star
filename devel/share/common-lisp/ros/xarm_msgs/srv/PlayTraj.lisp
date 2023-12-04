; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude PlayTraj-request.msg.html

(cl:defclass <PlayTraj-request> (roslisp-msg-protocol:ros-message)
  ((traj_file
    :reader traj_file
    :initarg :traj_file
    :type cl:string
    :initform "")
   (repeat_times
    :reader repeat_times
    :initarg :repeat_times
    :type cl:fixnum
    :initform 0)
   (speed_factor
    :reader speed_factor
    :initarg :speed_factor
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PlayTraj-request (<PlayTraj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayTraj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayTraj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<PlayTraj-request> is deprecated: use xarm_msgs-srv:PlayTraj-request instead.")))

(cl:ensure-generic-function 'traj_file-val :lambda-list '(m))
(cl:defmethod traj_file-val ((m <PlayTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:traj_file-val is deprecated.  Use xarm_msgs-srv:traj_file instead.")
  (traj_file m))

(cl:ensure-generic-function 'repeat_times-val :lambda-list '(m))
(cl:defmethod repeat_times-val ((m <PlayTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:repeat_times-val is deprecated.  Use xarm_msgs-srv:repeat_times instead.")
  (repeat_times m))

(cl:ensure-generic-function 'speed_factor-val :lambda-list '(m))
(cl:defmethod speed_factor-val ((m <PlayTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:speed_factor-val is deprecated.  Use xarm_msgs-srv:speed_factor instead.")
  (speed_factor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayTraj-request>) ostream)
  "Serializes a message object of type '<PlayTraj-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'traj_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'traj_file))
  (cl:let* ((signed (cl:slot-value msg 'repeat_times)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed_factor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayTraj-request>) istream)
  "Deserializes a message object of type '<PlayTraj-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'traj_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'traj_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'repeat_times) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_factor) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayTraj-request>)))
  "Returns string type for a service object of type '<PlayTraj-request>"
  "xarm_msgs/PlayTrajRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTraj-request)))
  "Returns string type for a service object of type 'PlayTraj-request"
  "xarm_msgs/PlayTrajRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayTraj-request>)))
  "Returns md5sum for a message object of type '<PlayTraj-request>"
  "80c963774ceddb6d2f6133dfc972a9dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayTraj-request)))
  "Returns md5sum for a message object of type 'PlayTraj-request"
  "80c963774ceddb6d2f6133dfc972a9dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayTraj-request>)))
  "Returns full string definition for message of type '<PlayTraj-request>"
  (cl:format cl:nil "# request: set multiple integer parameters.~%# traj_file: file name of trajectory record, do not forget \".traj\" suffix~%# repeat_times: how many times will be played~%# speed_factor: supported values are 1 (original speed), 2 (2x speed), 4 (4x speed, not recommended). ~%~%string traj_file~%int16 repeat_times~%int16 speed_factor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayTraj-request)))
  "Returns full string definition for message of type 'PlayTraj-request"
  (cl:format cl:nil "# request: set multiple integer parameters.~%# traj_file: file name of trajectory record, do not forget \".traj\" suffix~%# repeat_times: how many times will be played~%# speed_factor: supported values are 1 (original speed), 2 (2x speed), 4 (4x speed, not recommended). ~%~%string traj_file~%int16 repeat_times~%int16 speed_factor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayTraj-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'traj_file))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayTraj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayTraj-request
    (cl:cons ':traj_file (traj_file msg))
    (cl:cons ':repeat_times (repeat_times msg))
    (cl:cons ':speed_factor (speed_factor msg))
))
;//! \htmlinclude PlayTraj-response.msg.html

(cl:defclass <PlayTraj-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass PlayTraj-response (<PlayTraj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayTraj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayTraj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<PlayTraj-response> is deprecated: use xarm_msgs-srv:PlayTraj-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <PlayTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PlayTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayTraj-response>) ostream)
  "Serializes a message object of type '<PlayTraj-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayTraj-response>) istream)
  "Deserializes a message object of type '<PlayTraj-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayTraj-response>)))
  "Returns string type for a service object of type '<PlayTraj-response>"
  "xarm_msgs/PlayTrajResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTraj-response)))
  "Returns string type for a service object of type 'PlayTraj-response"
  "xarm_msgs/PlayTrajResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayTraj-response>)))
  "Returns md5sum for a message object of type '<PlayTraj-response>"
  "80c963774ceddb6d2f6133dfc972a9dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayTraj-response)))
  "Returns md5sum for a message object of type 'PlayTraj-response"
  "80c963774ceddb6d2f6133dfc972a9dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayTraj-response>)))
  "Returns full string definition for message of type '<PlayTraj-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayTraj-response)))
  "Returns full string definition for message of type 'PlayTraj-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayTraj-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayTraj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayTraj-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayTraj)))
  'PlayTraj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayTraj)))
  'PlayTraj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTraj)))
  "Returns string type for a service object of type '<PlayTraj>"
  "xarm_msgs/PlayTraj")