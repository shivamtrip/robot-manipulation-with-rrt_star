; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetMultipleInts-request.msg.html

(cl:defclass <SetMultipleInts-request> (roslisp-msg-protocol:ros-message)
  ((data_int
    :reader data_int
    :initarg :data_int
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SetMultipleInts-request (<SetMultipleInts-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMultipleInts-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMultipleInts-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetMultipleInts-request> is deprecated: use xarm_msgs-srv:SetMultipleInts-request instead.")))

(cl:ensure-generic-function 'data_int-val :lambda-list '(m))
(cl:defmethod data_int-val ((m <SetMultipleInts-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:data_int-val is deprecated.  Use xarm_msgs-srv:data_int instead.")
  (data_int m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMultipleInts-request>) ostream)
  "Serializes a message object of type '<SetMultipleInts-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data_int))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data_int))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMultipleInts-request>) istream)
  "Deserializes a message object of type '<SetMultipleInts-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data_int) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data_int)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMultipleInts-request>)))
  "Returns string type for a service object of type '<SetMultipleInts-request>"
  "xarm_msgs/SetMultipleIntsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMultipleInts-request)))
  "Returns string type for a service object of type 'SetMultipleInts-request"
  "xarm_msgs/SetMultipleIntsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMultipleInts-request>)))
  "Returns md5sum for a message object of type '<SetMultipleInts-request>"
  "72a7c6b1ca8071c36cdc4733c55fc7f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMultipleInts-request)))
  "Returns md5sum for a message object of type 'SetMultipleInts-request"
  "72a7c6b1ca8071c36cdc4733c55fc7f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMultipleInts-request>)))
  "Returns full string definition for message of type '<SetMultipleInts-request>"
  (cl:format cl:nil "# request: set multiple integer parameters.~%# data_int: int values to be set.~%~%int16[] data_int~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMultipleInts-request)))
  "Returns full string definition for message of type 'SetMultipleInts-request"
  (cl:format cl:nil "# request: set multiple integer parameters.~%# data_int: int values to be set.~%~%int16[] data_int~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMultipleInts-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data_int) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMultipleInts-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMultipleInts-request
    (cl:cons ':data_int (data_int msg))
))
;//! \htmlinclude SetMultipleInts-response.msg.html

(cl:defclass <SetMultipleInts-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetMultipleInts-response (<SetMultipleInts-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMultipleInts-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMultipleInts-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetMultipleInts-response> is deprecated: use xarm_msgs-srv:SetMultipleInts-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetMultipleInts-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetMultipleInts-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMultipleInts-response>) ostream)
  "Serializes a message object of type '<SetMultipleInts-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMultipleInts-response>) istream)
  "Deserializes a message object of type '<SetMultipleInts-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMultipleInts-response>)))
  "Returns string type for a service object of type '<SetMultipleInts-response>"
  "xarm_msgs/SetMultipleIntsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMultipleInts-response)))
  "Returns string type for a service object of type 'SetMultipleInts-response"
  "xarm_msgs/SetMultipleIntsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMultipleInts-response>)))
  "Returns md5sum for a message object of type '<SetMultipleInts-response>"
  "72a7c6b1ca8071c36cdc4733c55fc7f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMultipleInts-response)))
  "Returns md5sum for a message object of type 'SetMultipleInts-response"
  "72a7c6b1ca8071c36cdc4733c55fc7f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMultipleInts-response>)))
  "Returns full string definition for message of type '<SetMultipleInts-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMultipleInts-response)))
  "Returns full string definition for message of type 'SetMultipleInts-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMultipleInts-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMultipleInts-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMultipleInts-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMultipleInts)))
  'SetMultipleInts-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMultipleInts)))
  'SetMultipleInts-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMultipleInts)))
  "Returns string type for a service object of type '<SetMultipleInts>"
  "xarm_msgs/SetMultipleInts")