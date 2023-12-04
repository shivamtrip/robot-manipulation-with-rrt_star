; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetToolModbus-request.msg.html

(cl:defclass <SetToolModbus-request> (roslisp-msg-protocol:ros-message)
  ((send_data
    :reader send_data
    :initarg :send_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (respond_len
    :reader respond_len
    :initarg :respond_len
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetToolModbus-request (<SetToolModbus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolModbus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolModbus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetToolModbus-request> is deprecated: use xarm_msgs-srv:SetToolModbus-request instead.")))

(cl:ensure-generic-function 'send_data-val :lambda-list '(m))
(cl:defmethod send_data-val ((m <SetToolModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:send_data-val is deprecated.  Use xarm_msgs-srv:send_data instead.")
  (send_data m))

(cl:ensure-generic-function 'respond_len-val :lambda-list '(m))
(cl:defmethod respond_len-val ((m <SetToolModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:respond_len-val is deprecated.  Use xarm_msgs-srv:respond_len instead.")
  (respond_len m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolModbus-request>) ostream)
  "Serializes a message object of type '<SetToolModbus-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'send_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'send_data))
  (cl:let* ((signed (cl:slot-value msg 'respond_len)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolModbus-request>) istream)
  "Deserializes a message object of type '<SetToolModbus-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'send_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'send_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'respond_len) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolModbus-request>)))
  "Returns string type for a service object of type '<SetToolModbus-request>"
  "xarm_msgs/SetToolModbusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolModbus-request)))
  "Returns string type for a service object of type 'SetToolModbus-request"
  "xarm_msgs/SetToolModbusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolModbus-request>)))
  "Returns md5sum for a message object of type '<SetToolModbus-request>"
  "0b8299eb42a5262c78587a39fca90c6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolModbus-request)))
  "Returns md5sum for a message object of type 'SetToolModbus-request"
  "0b8299eb42a5262c78587a39fca90c6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolModbus-request>)))
  "Returns full string definition for message of type '<SetToolModbus-request>"
  (cl:format cl:nil "# unsigned char data to be sent to tool device through modbus~%uint8[] send_data~%~%# Specify the anticipated maximum respond data length in bytes~%int16 respond_len~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolModbus-request)))
  "Returns full string definition for message of type 'SetToolModbus-request"
  (cl:format cl:nil "# unsigned char data to be sent to tool device through modbus~%uint8[] send_data~%~%# Specify the anticipated maximum respond data length in bytes~%int16 respond_len~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolModbus-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'send_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolModbus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolModbus-request
    (cl:cons ':send_data (send_data msg))
    (cl:cons ':respond_len (respond_len msg))
))
;//! \htmlinclude SetToolModbus-response.msg.html

(cl:defclass <SetToolModbus-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (respond_data
    :reader respond_data
    :initarg :respond_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SetToolModbus-response (<SetToolModbus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolModbus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolModbus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetToolModbus-response> is deprecated: use xarm_msgs-srv:SetToolModbus-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetToolModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetToolModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'respond_data-val :lambda-list '(m))
(cl:defmethod respond_data-val ((m <SetToolModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:respond_data-val is deprecated.  Use xarm_msgs-srv:respond_data instead.")
  (respond_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolModbus-response>) ostream)
  "Serializes a message object of type '<SetToolModbus-response>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'respond_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'respond_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolModbus-response>) istream)
  "Deserializes a message object of type '<SetToolModbus-response>"
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'respond_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'respond_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolModbus-response>)))
  "Returns string type for a service object of type '<SetToolModbus-response>"
  "xarm_msgs/SetToolModbusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolModbus-response)))
  "Returns string type for a service object of type 'SetToolModbus-response"
  "xarm_msgs/SetToolModbusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolModbus-response>)))
  "Returns md5sum for a message object of type '<SetToolModbus-response>"
  "0b8299eb42a5262c78587a39fca90c6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolModbus-response)))
  "Returns md5sum for a message object of type 'SetToolModbus-response"
  "0b8299eb42a5262c78587a39fca90c6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolModbus-response>)))
  "Returns full string definition for message of type '<SetToolModbus-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%uint8[] respond_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolModbus-response)))
  "Returns full string definition for message of type 'SetToolModbus-response"
  (cl:format cl:nil "~%int16 ret~%string message~%uint8[] respond_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolModbus-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'respond_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolModbus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolModbus-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
    (cl:cons ':respond_data (respond_data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetToolModbus)))
  'SetToolModbus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetToolModbus)))
  'SetToolModbus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolModbus)))
  "Returns string type for a service object of type '<SetToolModbus>"
  "xarm_msgs/SetToolModbus")