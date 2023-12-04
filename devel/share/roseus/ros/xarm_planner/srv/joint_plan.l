;; Auto-generated. Do not edit!


(when (boundp 'xarm_planner::joint_plan)
  (if (not (find-package "XARM_PLANNER"))
    (make-package "XARM_PLANNER"))
  (shadow 'joint_plan (find-package "XARM_PLANNER")))
(unless (find-package "XARM_PLANNER::JOINT_PLAN")
  (make-package "XARM_PLANNER::JOINT_PLAN"))
(unless (find-package "XARM_PLANNER::JOINT_PLANREQUEST")
  (make-package "XARM_PLANNER::JOINT_PLANREQUEST"))
(unless (find-package "XARM_PLANNER::JOINT_PLANRESPONSE")
  (make-package "XARM_PLANNER::JOINT_PLANRESPONSE"))

(in-package "ROS")





(defclass xarm_planner::joint_planRequest
  :super ros::object
  :slots (_target ))

(defmethod xarm_planner::joint_planRequest
  (:init
   (&key
    ((:target __target) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _target __target)
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:serialization-length
   ()
   (+
    ;; float64[] _target
    (* 8    (length _target)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _target
     (write-long (length _target) s)
     (dotimes (i (length _target))
       (sys::poke (elt _target i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _target
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _target i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass xarm_planner::joint_planResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_planner::joint_planResponse
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

(defclass xarm_planner::joint_plan
  :super ros::object
  :slots ())

(setf (get xarm_planner::joint_plan :md5sum-) "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(setf (get xarm_planner::joint_plan :datatype-) "xarm_planner/joint_plan")
(setf (get xarm_planner::joint_plan :request) xarm_planner::joint_planRequest)
(setf (get xarm_planner::joint_plan :response) xarm_planner::joint_planResponse)

(defmethod xarm_planner::joint_planRequest
  (:response () (instance xarm_planner::joint_planResponse :init)))

(setf (get xarm_planner::joint_planRequest :md5sum-) "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(setf (get xarm_planner::joint_planRequest :datatype-) "xarm_planner/joint_planRequest")
(setf (get xarm_planner::joint_planRequest :definition-)
      "# list of target joint positions in radian.
float64[] target

---

bool success
")

(setf (get xarm_planner::joint_planResponse :md5sum-) "80c5e1dbd8cca88d8f7d1c0e51bf50cf")
(setf (get xarm_planner::joint_planResponse :datatype-) "xarm_planner/joint_planResponse")
(setf (get xarm_planner::joint_planResponse :definition-)
      "# list of target joint positions in radian.
float64[] target

---

bool success
")



(provide :xarm_planner/joint_plan "80c5e1dbd8cca88d8f7d1c0e51bf50cf")


