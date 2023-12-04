
(cl:in-package :asdf)

(defsystem "xarm_gripper-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MoveAction" :depends-on ("_package_MoveAction"))
    (:file "_package_MoveAction" :depends-on ("_package"))
    (:file "MoveActionFeedback" :depends-on ("_package_MoveActionFeedback"))
    (:file "_package_MoveActionFeedback" :depends-on ("_package"))
    (:file "MoveActionGoal" :depends-on ("_package_MoveActionGoal"))
    (:file "_package_MoveActionGoal" :depends-on ("_package"))
    (:file "MoveActionResult" :depends-on ("_package_MoveActionResult"))
    (:file "_package_MoveActionResult" :depends-on ("_package"))
    (:file "MoveFeedback" :depends-on ("_package_MoveFeedback"))
    (:file "_package_MoveFeedback" :depends-on ("_package"))
    (:file "MoveGoal" :depends-on ("_package_MoveGoal"))
    (:file "_package_MoveGoal" :depends-on ("_package"))
    (:file "MoveResult" :depends-on ("_package_MoveResult"))
    (:file "_package_MoveResult" :depends-on ("_package"))
  ))