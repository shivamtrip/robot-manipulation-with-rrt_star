; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-msg)


;//! \htmlinclude CIOState.msg.html

(cl:defclass <CIOState> (roslisp-msg-protocol:ros-message)
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
   (code
    :reader code
    :initarg :code
    :type cl:fixnum
    :initform 0)
   (input_digitals
    :reader input_digitals
    :initarg :input_digitals
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0))
   (output_digitals
    :reader output_digitals
    :initarg :output_digitals
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0))
   (input_analogs
    :reader input_analogs
    :initarg :input_analogs
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (output_analogs
    :reader output_analogs
    :initarg :output_analogs
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (input_conf
    :reader input_conf
    :initarg :input_conf
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0))
   (output_conf
    :reader output_conf
    :initarg :output_conf
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass CIOState (<CIOState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CIOState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CIOState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-msg:<CIOState> is deprecated: use xarm_msgs-msg:CIOState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:header-val is deprecated.  Use xarm_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:state-val is deprecated.  Use xarm_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:code-val is deprecated.  Use xarm_msgs-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'input_digitals-val :lambda-list '(m))
(cl:defmethod input_digitals-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:input_digitals-val is deprecated.  Use xarm_msgs-msg:input_digitals instead.")
  (input_digitals m))

(cl:ensure-generic-function 'output_digitals-val :lambda-list '(m))
(cl:defmethod output_digitals-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:output_digitals-val is deprecated.  Use xarm_msgs-msg:output_digitals instead.")
  (output_digitals m))

(cl:ensure-generic-function 'input_analogs-val :lambda-list '(m))
(cl:defmethod input_analogs-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:input_analogs-val is deprecated.  Use xarm_msgs-msg:input_analogs instead.")
  (input_analogs m))

(cl:ensure-generic-function 'output_analogs-val :lambda-list '(m))
(cl:defmethod output_analogs-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:output_analogs-val is deprecated.  Use xarm_msgs-msg:output_analogs instead.")
  (output_analogs m))

(cl:ensure-generic-function 'input_conf-val :lambda-list '(m))
(cl:defmethod input_conf-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:input_conf-val is deprecated.  Use xarm_msgs-msg:input_conf instead.")
  (input_conf m))

(cl:ensure-generic-function 'output_conf-val :lambda-list '(m))
(cl:defmethod output_conf-val ((m <CIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-msg:output_conf-val is deprecated.  Use xarm_msgs-msg:output_conf instead.")
  (output_conf m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CIOState>) ostream)
  "Serializes a message object of type '<CIOState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'input_digitals))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'output_digitals))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'input_analogs))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'output_analogs))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'input_conf))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'output_conf))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CIOState>) istream)
  "Deserializes a message object of type '<CIOState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'input_digitals) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'input_digitals)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'output_digitals) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'output_digitals)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'input_analogs) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'input_analogs)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'output_analogs) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'output_analogs)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'input_conf) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'input_conf)))
    (cl:dotimes (i 16)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'output_conf) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'output_conf)))
    (cl:dotimes (i 16)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CIOState>)))
  "Returns string type for a message object of type '<CIOState>"
  "xarm_msgs/CIOState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CIOState)))
  "Returns string type for a message object of type 'CIOState"
  "xarm_msgs/CIOState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CIOState>)))
  "Returns md5sum for a message object of type '<CIOState>"
  "2a51ddec3163bc4c38eaa278474b2dc1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CIOState)))
  "Returns md5sum for a message object of type 'CIOState"
  "2a51ddec3163bc4c38eaa278474b2dc1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CIOState>)))
  "Returns full string definition for message of type '<CIOState>"
  (cl:format cl:nil "Header header~%~%# contorller gpio module state~%int16 state~%~%# controller gpio module error code~%int16 code~%~%# input_digitals[0]: digital input functional gpio state~%# input_digitals[1]: digital input configuring gpio state~%#    CI0: (input_digitals[1] >> 0) & 0x0001~%#    CI1: (input_digitals[1] >> 1) & 0x0001~%#    CI7: (input_digitals[1] >> 7) & 0x0001~%#    DI0: (input_digitals[1] >> 8) & 0x0001~%#    DI1: (input_digitals[1] >> 9) & 0x0001~%#    DI7: (input_digitals[1] >> 15) & 0x0001~%uint16[2] input_digitals~%~%# output_digitals[0]: digital output functional gpio state~%# output_digitals[1]: digital output configuring gpio state~%#    CO0: (output_digitals[1] >> 0) & 0x0001~%#    CO1: (output_digitals[1] >> 1) & 0x0001~%#    CO7: (output_digitals[1] >> 7) & 0x0001~%#    DO0: (output_digitals[1] >> 8) & 0x0001~%#    DO1: (output_digitals[1] >> 9) & 0x0001~%#    DO7: (output_digitals[1] >> 15) & 0x0001~%uint16[2] output_digitals~%~%# input_analogs[0]: the value of AI0~%# input_analogs[1]: the value of AI1~%float32[2] input_analogs~%~%# output_analogs[0]: the value of AO0~%# output_analogs[1]: the value of AO1~%float32[2] output_analogs~%~%# digital input functional info~%uint8[16] input_conf~%~%# digital output functional info~%uint8[16] output_conf~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CIOState)))
  "Returns full string definition for message of type 'CIOState"
  (cl:format cl:nil "Header header~%~%# contorller gpio module state~%int16 state~%~%# controller gpio module error code~%int16 code~%~%# input_digitals[0]: digital input functional gpio state~%# input_digitals[1]: digital input configuring gpio state~%#    CI0: (input_digitals[1] >> 0) & 0x0001~%#    CI1: (input_digitals[1] >> 1) & 0x0001~%#    CI7: (input_digitals[1] >> 7) & 0x0001~%#    DI0: (input_digitals[1] >> 8) & 0x0001~%#    DI1: (input_digitals[1] >> 9) & 0x0001~%#    DI7: (input_digitals[1] >> 15) & 0x0001~%uint16[2] input_digitals~%~%# output_digitals[0]: digital output functional gpio state~%# output_digitals[1]: digital output configuring gpio state~%#    CO0: (output_digitals[1] >> 0) & 0x0001~%#    CO1: (output_digitals[1] >> 1) & 0x0001~%#    CO7: (output_digitals[1] >> 7) & 0x0001~%#    DO0: (output_digitals[1] >> 8) & 0x0001~%#    DO1: (output_digitals[1] >> 9) & 0x0001~%#    DO7: (output_digitals[1] >> 15) & 0x0001~%uint16[2] output_digitals~%~%# input_analogs[0]: the value of AI0~%# input_analogs[1]: the value of AI1~%float32[2] input_analogs~%~%# output_analogs[0]: the value of AO0~%# output_analogs[1]: the value of AO1~%float32[2] output_analogs~%~%# digital input functional info~%uint8[16] input_conf~%~%# digital output functional info~%uint8[16] output_conf~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CIOState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'input_digitals) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'output_digitals) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'input_analogs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'output_analogs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'input_conf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'output_conf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CIOState>))
  "Converts a ROS message object to a list"
  (cl:list 'CIOState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':code (code msg))
    (cl:cons ':input_digitals (input_digitals msg))
    (cl:cons ':output_digitals (output_digitals msg))
    (cl:cons ':input_analogs (input_analogs msg))
    (cl:cons ':output_analogs (output_analogs msg))
    (cl:cons ':input_conf (input_conf msg))
    (cl:cons ':output_conf (output_conf msg))
))
