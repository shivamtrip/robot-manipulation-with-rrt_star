; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude FtCaliLoad-request.msg.html

(cl:defclass <FtCaliLoad-request> (roslisp-msg-protocol:ros-message)
  ((datas
    :reader datas
    :initarg :datas
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (association_setting_tcp_load
    :reader association_setting_tcp_load
    :initarg :association_setting_tcp_load
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FtCaliLoad-request (<FtCaliLoad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtCaliLoad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtCaliLoad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<FtCaliLoad-request> is deprecated: use xarm_msgs-srv:FtCaliLoad-request instead.")))

(cl:ensure-generic-function 'datas-val :lambda-list '(m))
(cl:defmethod datas-val ((m <FtCaliLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:datas-val is deprecated.  Use xarm_msgs-srv:datas instead.")
  (datas m))

(cl:ensure-generic-function 'association_setting_tcp_load-val :lambda-list '(m))
(cl:defmethod association_setting_tcp_load-val ((m <FtCaliLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:association_setting_tcp_load-val is deprecated.  Use xarm_msgs-srv:association_setting_tcp_load instead.")
  (association_setting_tcp_load m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtCaliLoad-request>) ostream)
  "Serializes a message object of type '<FtCaliLoad-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'datas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'datas))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'association_setting_tcp_load) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtCaliLoad-request>) istream)
  "Deserializes a message object of type '<FtCaliLoad-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'datas) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'datas)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'association_setting_tcp_load) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtCaliLoad-request>)))
  "Returns string type for a service object of type '<FtCaliLoad-request>"
  "xarm_msgs/FtCaliLoadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtCaliLoad-request)))
  "Returns string type for a service object of type 'FtCaliLoad-request"
  "xarm_msgs/FtCaliLoadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtCaliLoad-request>)))
  "Returns md5sum for a message object of type '<FtCaliLoad-request>"
  "002aec91323cfbf6a70a5f91f59ae0cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtCaliLoad-request)))
  "Returns md5sum for a message object of type 'FtCaliLoad-request"
  "002aec91323cfbf6a70a5f91f59ae0cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtCaliLoad-request>)))
  "Returns full string definition for message of type '<FtCaliLoad-request>"
  (cl:format cl:nil "# This format is suitable for the following services~%#   - ft_sensor_cali_load~%~%# iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])~%float32[] datas~%~%# whether to convert the paramster to the corresponding tcp load and set~%bool association_setting_tcp_load~%# float32 m   0.325~%# float32 x   -17.0~%# float32 y   9.0~%# float32 z   11.8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtCaliLoad-request)))
  "Returns full string definition for message of type 'FtCaliLoad-request"
  (cl:format cl:nil "# This format is suitable for the following services~%#   - ft_sensor_cali_load~%~%# iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])~%float32[] datas~%~%# whether to convert the paramster to the corresponding tcp load and set~%bool association_setting_tcp_load~%# float32 m   0.325~%# float32 x   -17.0~%# float32 y   9.0~%# float32 z   11.8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtCaliLoad-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'datas) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtCaliLoad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FtCaliLoad-request
    (cl:cons ':datas (datas msg))
    (cl:cons ':association_setting_tcp_load (association_setting_tcp_load msg))
))
;//! \htmlinclude FtCaliLoad-response.msg.html

(cl:defclass <FtCaliLoad-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass FtCaliLoad-response (<FtCaliLoad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtCaliLoad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtCaliLoad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<FtCaliLoad-response> is deprecated: use xarm_msgs-srv:FtCaliLoad-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <FtCaliLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <FtCaliLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtCaliLoad-response>) ostream)
  "Serializes a message object of type '<FtCaliLoad-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtCaliLoad-response>) istream)
  "Deserializes a message object of type '<FtCaliLoad-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtCaliLoad-response>)))
  "Returns string type for a service object of type '<FtCaliLoad-response>"
  "xarm_msgs/FtCaliLoadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtCaliLoad-response)))
  "Returns string type for a service object of type 'FtCaliLoad-response"
  "xarm_msgs/FtCaliLoadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtCaliLoad-response>)))
  "Returns md5sum for a message object of type '<FtCaliLoad-response>"
  "002aec91323cfbf6a70a5f91f59ae0cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtCaliLoad-response)))
  "Returns md5sum for a message object of type 'FtCaliLoad-response"
  "002aec91323cfbf6a70a5f91f59ae0cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtCaliLoad-response>)))
  "Returns full string definition for message of type '<FtCaliLoad-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtCaliLoad-response)))
  "Returns full string definition for message of type 'FtCaliLoad-response"
  (cl:format cl:nil "~%int16 ret~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtCaliLoad-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtCaliLoad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FtCaliLoad-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FtCaliLoad)))
  'FtCaliLoad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FtCaliLoad)))
  'FtCaliLoad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtCaliLoad)))
  "Returns string type for a service object of type '<FtCaliLoad>"
  "xarm_msgs/FtCaliLoad")