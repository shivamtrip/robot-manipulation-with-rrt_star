;; Auto-generated. Do not edit!


(when (boundp 'xarm_planner::exec_plan)
  (if (not (find-package "XARM_PLANNER"))
    (make-package "XARM_PLANNER"))
  (shadow 'exec_plan (find-package "XARM_PLANNER")))
(unless (find-package "XARM_PLANNER::EXEC_PLAN")
  (make-package "XARM_PLANNER::EXEC_PLAN"))
(unless (find-package "XARM_PLANNER::EXEC_PLANREQUEST")
  (make-package "XARM_PLANNER::EXEC_PLANREQUEST"))
(unless (find-package "XARM_PLANNER::EXEC_PLANRESPONSE")
  (make-package "XARM_PLANNER::EXEC_PLANRESPONSE"))

(in-package "ROS")





(defclass xarm_planner::exec_planRequest
  :super ros::object
  :slots (_exec ))

(defmethod xarm_planner::exec_planRequest
  (:init
   (&key
    ((:exec __exec) nil)
    )
   (send-super :init)
   (setq _exec __exec)
   self)
  (:exec
   (&optional (__exec :null))
   (if (not (eq __exec :null)) (setq _exec __exec)) _exec)
  (:serialization-length
   ()
   (+
    ;; bool _exec
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _exec
       (if _exec (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _exec
     (setq _exec (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_planner::exec_planResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_planner::exec_planResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_planner::exec_plan
  :super ros::object
  :slots ())

(setf (get xarm_planner::exec_plan :md5sum-) "79a98935480e148cf8a7f293507dd090")
(setf (get xarm_planner::exec_plan :datatype-) "xarm_planner/exec_plan")
(setf (get xarm_planner::exec_plan :request) xarm_planner::exec_planRequest)
(setf (get xarm_planner::exec_plan :response) xarm_planner::exec_planResponse)

(defmethod xarm_planner::exec_planRequest
  (:response () (instance xarm_planner::exec_planResponse :init)))

(setf (get xarm_planner::exec_planRequest :md5sum-) "79a98935480e148cf8a7f293507dd090")
(setf (get xarm_planner::exec_planRequest :datatype-) "xarm_planner/exec_planRequest")
(setf (get xarm_planner::exec_planRequest :definition-)
      "bool exec
---
bool success
")

(setf (get xarm_planner::exec_planResponse :md5sum-) "79a98935480e148cf8a7f293507dd090")
(setf (get xarm_planner::exec_planResponse :datatype-) "xarm_planner/exec_planResponse")
(setf (get xarm_planner::exec_planResponse :definition-)
      "bool exec
---
bool success
")



(provide :xarm_planner/exec_plan "79a98935480e148cf8a7f293507dd090")


