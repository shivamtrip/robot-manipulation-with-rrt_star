
(cl:in-package :asdf)

(defsystem "xarm_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CIOState" :depends-on ("_package_CIOState"))
    (:file "_package_CIOState" :depends-on ("_package"))
    (:file "IOState" :depends-on ("_package_IOState"))
    (:file "_package_IOState" :depends-on ("_package"))
    (:file "RobotMsg" :depends-on ("_package_RobotMsg"))
    (:file "_package_RobotMsg" :depends-on ("_package"))
  ))