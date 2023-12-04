
(cl:in-package :asdf)

(defsystem "xarm_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "exec_plan" :depends-on ("_package_exec_plan"))
    (:file "_package_exec_plan" :depends-on ("_package"))
    (:file "joint_plan" :depends-on ("_package_joint_plan"))
    (:file "_package_joint_plan" :depends-on ("_package"))
    (:file "pose_plan" :depends-on ("_package_pose_plan"))
    (:file "_package_pose_plan" :depends-on ("_package"))
    (:file "single_straight_plan" :depends-on ("_package_single_straight_plan"))
    (:file "_package_single_straight_plan" :depends-on ("_package"))
  ))