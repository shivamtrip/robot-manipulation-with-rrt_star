; Auto-generated. Do not edit!


(cl:in-package xarm_planner-srv)


;//! \htmlinclude exec_plan-request.msg.html

(cl:defclass <exec_plan-request> (roslisp-msg-protocol:ros-message)
  ((exec
    :reader exec
    :initarg :exec
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass exec_plan-request (<exec_plan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <exec_plan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'exec_plan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_planner-srv:<exec_plan-request> is deprecated: use xarm_planner-srv:exec_plan-request instead.")))

(cl:ensure-generic-function 'exec-val :lambda-list '(m))
(cl:defmethod exec-val ((m <exec_plan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_planner-srv:exec-val is deprecated.  Use xarm_planner-srv:exec instead.")
  (exec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <exec_plan-request>) ostream)
  "Serializes a message object of type '<exec_plan-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exec) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <exec_plan-request>) istream)
  "Deserializes a message object of type '<exec_plan-request>"
    (cl:setf (cl:slot-value msg 'exec) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<exec_plan-request>)))
  "Returns string type for a service object of type '<exec_plan-request>"
  "xarm_planner/exec_planRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'exec_plan-request)))
  "Returns string type for a service object of type 'exec_plan-request"
  "xarm_planner/exec_planRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<exec_plan-request>)))
  "Returns md5sum for a message object of type '<exec_plan-request>"
  "79a98935480e148cf8a7f293507dd090")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'exec_plan-request)))
  "Returns md5sum for a message object of type 'exec_plan-request"
  "79a98935480e148cf8a7f293507dd090")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<exec_plan-request>)))
  "Returns full string definition for message of type '<exec_plan-request>"
  (cl:format cl:nil "bool exec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'exec_plan-request)))
  "Returns full string definition for message of type 'exec_plan-request"
  (cl:format cl:nil "bool exec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <exec_plan-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <exec_plan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'exec_plan-request
    (cl:cons ':exec (exec msg))
))
;//! \htmlinclude exec_plan-response.msg.html

(cl:defclass <exec_plan-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass exec_plan-response (<exec_plan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <exec_plan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'exec_plan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xarm_planner-srv:<exec_plan-response> is deprecated: use xarm_planner-srv:exec_plan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <exec_plan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xarm_planner-srv:success-val is deprecated.  Use xarm_planner-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <exec_plan-response>) ostream)
  "Serializes a message object of type '<exec_plan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <exec_plan-response>) istream)
  "Deserializes a message object of type '<exec_plan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<exec_plan-response>)))
  "Returns string type for a service object of type '<exec_plan-response>"
  "xarm_planner/exec_planResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'exec_plan-response)))
  "Returns string type for a service object of type 'exec_plan-response"
  "xarm_planner/exec_planResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<exec_plan-response>)))
  "Returns md5sum for a message object of type '<exec_plan-response>"
  "79a98935480e148cf8a7f293507dd090")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'exec_plan-response)))
  "Returns md5sum for a message object of type 'exec_plan-response"
  "79a98935480e148cf8a7f293507dd090")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<exec_plan-response>)))
  "Returns full string definition for message of type '<exec_plan-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'exec_plan-response)))
  "Returns full string definition for message of type 'exec_plan-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <exec_plan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <exec_plan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'exec_plan-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'exec_plan)))
  'exec_plan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'exec_plan)))
  'exec_plan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'exec_plan)))
  "Returns string type for a service object of type '<exec_plan>"
  "xarm_planner/exec_plan")