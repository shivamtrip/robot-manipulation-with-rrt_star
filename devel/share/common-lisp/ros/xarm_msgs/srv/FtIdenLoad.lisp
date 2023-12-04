; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude FtIdenLoad-request.msg.html

(cl:defclass <FtIdenLoad-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FtIdenLoad-request (<FtIdenLoad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtIdenLoad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtIdenLoad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<FtIdenLoad-request> is deprecated: use xarm_msgs-srv:FtIdenLoad-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtIdenLoad-request>) ostream)
  "Serializes a message object of type '<FtIdenLoad-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtIdenLoad-request>) istream)
  "Deserializes a message object of type '<FtIdenLoad-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtIdenLoad-request>)))
  "Returns string type for a service object of type '<FtIdenLoad-request>"
  "xarm_msgs/FtIdenLoadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtIdenLoad-request)))
  "Returns string type for a service object of type 'FtIdenLoad-request"
  "xarm_msgs/FtIdenLoadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtIdenLoad-request>)))
  "Returns md5sum for a message object of type '<FtIdenLoad-request>"
  "9470c18b0944f559a19d029d524ee6b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtIdenLoad-request)))
  "Returns md5sum for a message object of type 'FtIdenLoad-request"
  "9470c18b0944f559a19d029d524ee6b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtIdenLoad-request>)))
  "Returns full string definition for message of type '<FtIdenLoad-request>"
  (cl:format cl:nil "# This format is suitable for the following services~%#   - ft_sensor_iden_load~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtIdenLoad-request)))
  "Returns full string definition for message of type 'FtIdenLoad-request"
  (cl:format cl:nil "# This format is suitable for the following services~%#   - ft_sensor_iden_load~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtIdenLoad-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtIdenLoad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FtIdenLoad-request
))
;//! \htmlinclude FtIdenLoad-response.msg.html

(cl:defclass <FtIdenLoad-response> (roslisp-msg-protocol:ros-message)
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
   (datas
    :reader datas
    :initarg :datas
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass FtIdenLoad-response (<FtIdenLoad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtIdenLoad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtIdenLoad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<FtIdenLoad-response> is deprecated: use xarm_msgs-srv:FtIdenLoad-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <FtIdenLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <FtIdenLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'datas-val :lambda-list '(m))
(cl:defmethod datas-val ((m <FtIdenLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:datas-val is deprecated.  Use xarm_msgs-srv:datas instead.")
  (datas m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtIdenLoad-response>) ostream)
  "Serializes a message object of type '<FtIdenLoad-response>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtIdenLoad-response>) istream)
  "Deserializes a message object of type '<FtIdenLoad-response>"
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
  (cl:setf (cl:slot-value msg 'datas) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'datas)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtIdenLoad-response>)))
  "Returns string type for a service object of type '<FtIdenLoad-response>"
  "xarm_msgs/FtIdenLoadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtIdenLoad-response)))
  "Returns string type for a service object of type 'FtIdenLoad-response"
  "xarm_msgs/FtIdenLoadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtIdenLoad-response>)))
  "Returns md5sum for a message object of type '<FtIdenLoad-response>"
  "9470c18b0944f559a19d029d524ee6b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtIdenLoad-response)))
  "Returns md5sum for a message object of type 'FtIdenLoad-response"
  "9470c18b0944f559a19d029d524ee6b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtIdenLoad-response>)))
  "Returns full string definition for message of type '<FtIdenLoad-response>"
  (cl:format cl:nil "~%int16 ret~%string message~%~%# the result of identification~%#   ft_sensor_iden_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]~%float32[] datas~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtIdenLoad-response)))
  "Returns full string definition for message of type 'FtIdenLoad-response"
  (cl:format cl:nil "~%int16 ret~%string message~%~%# the result of identification~%#   ft_sensor_iden_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]~%float32[] datas~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtIdenLoad-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'datas) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtIdenLoad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FtIdenLoad-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
    (cl:cons ':datas (datas msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FtIdenLoad)))
  'FtIdenLoad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FtIdenLoad)))
  'FtIdenLoad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtIdenLoad)))
  "Returns string type for a service object of type '<FtIdenLoad>"
  "xarm_msgs/FtIdenLoad")