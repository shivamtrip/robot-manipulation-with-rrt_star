; Auto-generated. Do not edit!


(cl:in-package xarm_planner-srv)


;//! \htmlinclude joint_plan-request.msg.html

(cl:defclass <joint_plan-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass joint_plan-request (<joint_plan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_plan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_plan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_planner-srv:<joint_plan-request> is deprecated: use xarm_planner-srv:joint_plan-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <joint_plan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_planner-srv:target-val is deprecated.  Use xarm_planner-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_plan-request>) ostream)
  "Serializes a message object of type '<joint_plan-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'target))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_plan-request>) istream)
  "Deserializes a message object of type '<joint_plan-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_plan-request>)))
  "Returns string type for a service object of type '<joint_plan-request>"
  "xarm_planner/joint_planRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_plan-request)))
  "Returns string type for a service object of type 'joint_plan-request"
  "xarm_planner/joint_planRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_plan-request>)))
  "Returns md5sum for a message object of type '<joint_plan-request>"
  "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_plan-request)))
  "Returns md5sum for a message object of type 'joint_plan-request"
  "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_plan-request>)))
  "Returns full string definition for message of type '<joint_plan-request>"
  (cl:format cl:nil "# list of target joint positions in radian.~%float64[] target~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_plan-request)))
  "Returns full string definition for message of type 'joint_plan-request"
  (cl:format cl:nil "# list of target joint positions in radian.~%float64[] target~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_plan-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_plan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_plan-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude joint_plan-response.msg.html

(cl:defclass <joint_plan-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass joint_plan-response (<joint_plan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_plan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_plan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_planner-srv:<joint_plan-response> is deprecated: use xarm_planner-srv:joint_plan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <joint_plan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_planner-srv:success-val is deprecated.  Use xarm_planner-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_plan-response>) ostream)
  "Serializes a message object of type '<joint_plan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_plan-response>) istream)
  "Deserializes a message object of type '<joint_plan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_plan-response>)))
  "Returns string type for a service object of type '<joint_plan-response>"
  "xarm_planner/joint_planResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_plan-response)))
  "Returns string type for a service object of type 'joint_plan-response"
  "xarm_planner/joint_planResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_plan-response>)))
  "Returns md5sum for a message object of type '<joint_plan-response>"
  "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_plan-response)))
  "Returns md5sum for a message object of type 'joint_plan-response"
  "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_plan-response>)))
  "Returns full string definition for message of type '<joint_plan-response>"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_plan-response)))
  "Returns full string definition for message of type 'joint_plan-response"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_plan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_plan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_plan-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'joint_plan)))
  'joint_plan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'joint_plan)))
  'joint_plan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_plan)))
  "Returns string type for a service object of type '<joint_plan>"
  "xarm_planner/joint_plan")