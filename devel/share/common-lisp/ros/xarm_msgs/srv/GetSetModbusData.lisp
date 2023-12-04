; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude GetSetModbusData-request.msg.html

(cl:defclass <GetSetModbusData-request> (roslisp-msg-protocol:ros-message)
  ((send_data
    :reader send_data
    :initarg :send_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (respond_len
    :reader respond_len
    :initarg :respond_len
    :type cl:fixnum
    :initform 0)
   (host_id
    :reader host_id
    :initarg :host_id
    :type cl:fixnum
    :initform 0)
   (is_transparent_transmission
    :reader is_transparent_transmission
    :initarg :is_transparent_transmission
    :type cl:boolean
    :initform cl:nil)
   (use_503_port
    :reader use_503_port
    :initarg :use_503_port
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetSetModbusData-request (<GetSetModbusData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSetModbusData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSetModbusData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetSetModbusData-request> is deprecated: use xarm_msgs-srv:GetSetModbusData-request instead.")))

(cl:ensure-generic-function 'send_data-val :lambda-list '(m))
(cl:defmethod send_data-val ((m <GetSetModbusData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:send_data-val is deprecated.  Use xarm_msgs-srv:send_data instead.")
  (send_data m))

(cl:ensure-generic-function 'respond_len-val :lambda-list '(m))
(cl:defmethod respond_len-val ((m <GetSetModbusData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:respond_len-val is deprecated.  Use xarm_msgs-srv:respond_len instead.")
  (respond_len m))

(cl:ensure-generic-function 'host_id-val :lambda-list '(m))
(cl:defmethod host_id-val ((m <GetSetModbusData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:host_id-val is deprecated.  Use xarm_msgs-srv:host_id instead.")
  (host_id m))

(cl:ensure-generic-function 'is_transparent_transmission-val :lambda-list '(m))
(cl:defmethod is_transparent_transmission-val ((m <GetSetModbusData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:is_transparent_transmission-val is deprecated.  Use xarm_msgs-srv:is_transparent_transmission instead.")
  (is_transparent_transmission m))

(cl:ensure-generic-function 'use_503_port-val :lambda-list '(m))
(cl:defmethod use_503_port-val ((m <GetSetModbusData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:use_503_port-val is deprecated.  Use xarm_msgs-srv:use_503_port instead.")
  (use_503_port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSetModbusData-request>) ostream)
  "Serializes a message object of type '<GetSetModbusData-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'host_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_transparent_transmission) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_503_port) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSetModbusData-request>) istream)
  "Deserializes a message object of type '<GetSetModbusData-request>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'host_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_transparent_transmission) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'use_503_port) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSetModbusData-request>)))
  "Returns string type for a service object of type '<GetSetModbusData-request>"
  "xarm_msgs/GetSetModbusDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSetModbusData-request)))
  "Returns string type for a service object of type 'GetSetModbusData-request"
  "xarm_msgs/GetSetModbusDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSetModbusData-request>)))
  "Returns md5sum for a message object of type '<GetSetModbusData-request>"
  "dc342acff6bdfadcdcdb95fac36d565f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSetModbusData-request)))
  "Returns md5sum for a message object of type 'GetSetModbusData-request"
  "dc342acff6bdfadcdcdb95fac36d565f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSetModbusData-request>)))
  "Returns full string definition for message of type '<GetSetModbusData-request>"
  (cl:format cl:nil "# unsigned char data to be sent to tool device through modbus~%uint8[] send_data~%~%# Specify the anticipated maximum respond data length in bytes~%int16 respond_len~%~%# host id, 9: END RS485, 10: Controller RS485~%uint8 host_id~%~%# whether to choose transparent transmission~%bool is_transparent_transmission~%~%# whether to use port 503 for communication~%# if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission~%bool use_503_port~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSetModbusData-request)))
  "Returns full string definition for message of type 'GetSetModbusData-request"
  (cl:format cl:nil "# unsigned char data to be sent to tool device through modbus~%uint8[] send_data~%~%# Specify the anticipated maximum respond data length in bytes~%int16 respond_len~%~%# host id, 9: END RS485, 10: Controller RS485~%uint8 host_id~%~%# whether to choose transparent transmission~%bool is_transparent_transmission~%~%# whether to use port 503 for communication~%# if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission~%bool use_503_port~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSetModbusData-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'send_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     2
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSetModbusData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSetModbusData-request
    (cl:cons ':send_data (send_data msg))
    (cl:cons ':respond_len (respond_len msg))
    (cl:cons ':host_id (host_id msg))
    (cl:cons ':is_transparent_transmission (is_transparent_transmission msg))
    (cl:cons ':use_503_port (use_503_port msg))
))
;//! \htmlinclude GetSetModbusData-response.msg.html

(cl:defclass <GetSetModbusData-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetSetModbusData-response (<GetSetModbusData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSetModbusData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSetModbusData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<GetSetModbusData-response> is deprecated: use xarm_msgs-srv:GetSetModbusData-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <GetSetModbusData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetSetModbusData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'respond_data-val :lambda-list '(m))
(cl:defmethod respond_data-val ((m <GetSetModbusData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:respond_data-val is deprecated.  Use xarm_msgs-srv:respond_data instead.")
  (respond_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSetModbusData-response>) ostream)
  "Serializes a message object of type '<GetSetModbusData-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSetModbusData-response>) istream)
  "Deserializes a message object of type '<GetSetModbusData-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSetModbusData-response>)))
  "Returns string type for a service object of type '<GetSetModbusData-response>"
  "xarm_msgs/GetSetModbusDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSetModbusData-response)))
  "Returns string type for a service object of type 'GetSetModbusData-response"
  "xarm_msgs/GetSetModbusDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSetModbusData-response>)))
  "Returns md5sum for a message object of type '<GetSetModbusData-response>"
  "dc342acff6bdfadcdcdb95fac36d565f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSetModbusData-response)))
  "Returns md5sum for a message object of type 'GetSetModbusData-response"
  "dc342acff6bdfadcdcdb95fac36d565f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSetModbusData-response>)))
  "Returns full string definition for message of type '<GetSetModbusData-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%uint8[] respond_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSetModbusData-response)))
  "Returns full string definition for message of type 'GetSetModbusData-response"
  (cl:format cl:nil "~%int16 ret~%string message~%uint8[] respond_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSetModbusData-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'respond_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSetModbusData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSetModbusData-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
    (cl:cons ':respond_data (respond_data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSetModbusData)))
  'GetSetModbusData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSetModbusData)))
  'GetSetModbusData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSetModbusData)))
  "Returns string type for a service object of type '<GetSetModbusData>"
  "xarm_msgs/GetSetModbusData")