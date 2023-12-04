; Auto-generated. Do not edit!


(cl:in-package xarm_msgs-srv)


;//! \htmlinclude SetString-request.msg.html

(cl:defclass <SetString-request> (roslisp-msg-protocol:ros-message)
  ((str_data
    :reader str_data
    :initarg :str_data
    :type cl:string
    :initform "")
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetString-request (<SetString-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetString-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetString-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetString-request> is deprecated: use xarm_msgs-srv:SetString-request instead.")))

(cl:ensure-generic-function 'str_data-val :lambda-list '(m))
(cl:defmethod str_data-val ((m <SetString-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:str_data-val is deprecated.  Use xarm_msgs-srv:str_data instead.")
  (str_data m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <SetString-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:timeout-val is deprecated.  Use xarm_msgs-srv:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetString-request>) ostream)
  "Serializes a message object of type '<SetString-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str_data))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetString-request>) istream)
  "Deserializes a message object of type '<SetString-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetString-request>)))
  "Returns string type for a service object of type '<SetString-request>"
  "xarm_msgs/SetStringRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetString-request)))
  "Returns string type for a service object of type 'SetString-request"
  "xarm_msgs/SetStringRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetString-request>)))
  "Returns md5sum for a message object of type '<SetString-request>"
  "8ef44a329258a19c2b77a7a598761c55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetString-request)))
  "Returns md5sum for a message object of type 'SetString-request"
  "8ef44a329258a19c2b77a7a598761c55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetString-request>)))
  "Returns full string definition for message of type '<SetString-request>"
  (cl:format cl:nil "# request: set string command parameters.~%# str_data: string parameter to be set.~%~%string str_data~%float32 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetString-request)))
  "Returns full string definition for message of type 'SetString-request"
  (cl:format cl:nil "# request: set string command parameters.~%# str_data: string parameter to be set.~%~%string str_data~%float32 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetString-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str_data))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetString-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetString-request
    (cl:cons ':str_data (str_data msg))
    (cl:cons ':timeout (timeout msg))
))
;//! \htmlinclude SetString-response.msg.html

(cl:defclass <SetString-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetString-response (<SetString-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetString-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetString-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_msgs-srv:<SetString-response> is deprecated: use xarm_msgs-srv:SetString-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <SetString-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:ret-val is deprecated.  Use xarm_msgs-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetString-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_msgs-srv:message-val is deprecated.  Use xarm_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetString-response>) ostream)
  "Serializes a message object of type '<SetString-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetString-response>) istream)
  "Deserializes a message object of type '<SetString-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetString-response>)))
  "Returns string type for a service object of type '<SetString-response>"
  "xarm_msgs/SetStringResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetString-response)))
  "Returns string type for a service object of type 'SetString-response"
  "xarm_msgs/SetStringResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetString-response>)))
  "Returns md5sum for a message object of type '<SetString-response>"
  "8ef44a329258a19c2b77a7a598761c55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetString-response)))
  "Returns md5sum for a message object of type 'SetString-response"
  "8ef44a329258a19c2b77a7a598761c55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetString-response>)))
  "Returns full string definition for message of type '<SetString-response>"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetString-response)))
  "Returns full string definition for message of type 'SetString-response"
  (cl:format cl:nil "~%# response: ~%#	ret is 0 for successful execution and others for errors or warnings occured~%#	message is a string returned by function, indicating execution status.~%~%int16 ret~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetString-response>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetString-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetString-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetString)))
  'SetString-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetString)))
  'SetString-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetString)))
  "Returns string type for a service object of type '<SetString>"
  "xarm_msgs/SetString")