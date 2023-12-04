;; Auto-generated. Do not edit!


(when (boundp 'xarm_planner::pose_plan)
  (if (not (find-package "XARM_PLANNER"))
    (make-package "XARM_PLANNER"))
  (shadow 'pose_plan (find-package "XARM_PLANNER")))
(unless (find-package "XARM_PLANNER::POSE_PLAN")
  (make-package "XARM_PLANNER::POSE_PLAN"))
(unless (find-package "XARM_PLANNER::POSE_PLANREQUEST")
  (make-package "XARM_PLANNER::POSE_PLANREQUEST"))
(unless (find-package "XARM_PLANNER::POSE_PLANRESPONSE")
  (make-package "XARM_PLANNER::POSE_PLANRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass xarm_planner::pose_planRequest
  :super ros::object
  :slots (_target ))

(defmethod xarm_planner::pose_planRequest
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _target __target)
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target
    (send _target :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target
       (send _target :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;;
   self)
  )

(defclass xarm_planner::pose_planResponse
  :super ros::object
  :slots (_success ))

(defmethod xarm_planner::pose_planResponse
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

(defclass xarm_planner::pose_plan
  :super ros::object
  :slots ())

(setf (get xarm_planner::pose_plan :md5sum-) "9b6cd400aefab13efff1a5082333de0f")
(setf (get xarm_planner::pose_plan :datatype-) "xarm_planner/pose_plan")
(setf (get xarm_planner::pose_plan :request) xarm_planner::pose_planRequest)
(setf (get xarm_planner::pose_plan :response) xarm_planner::pose_planResponse)

(defmethod xarm_planner::pose_planRequest
  (:response () (instance xarm_planner::pose_planResponse :init)))

(setf (get xarm_planner::pose_planRequest :md5sum-) "9b6cd400aefab13efff1a5082333de0f")
(setf (get xarm_planner::pose_planRequest :datatype-) "xarm_planner/pose_planRequest")
(setf (get xarm_planner::pose_planRequest :definition-)
      "geometry_msgs/Pose target


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

bool success
")

(setf (get xarm_planner::pose_planResponse :md5sum-) "9b6cd400aefab13efff1a5082333de0f")
(setf (get xarm_planner::pose_planResponse :datatype-) "xarm_planner/pose_planResponse")
(setf (get xarm_planner::pose_planResponse :definition-)
      "geometry_msgs/Pose target


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

bool success
")



(provide :xarm_planner/pose_plan "9b6cd400aefab13efff1a5082333de0f")


