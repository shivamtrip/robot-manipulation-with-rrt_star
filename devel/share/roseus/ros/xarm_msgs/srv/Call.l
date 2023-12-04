;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::Call)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'Call (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::CALL")
  (make-package "XARM_MSGS::CALL"))
(unless (find-package "XARM_MSGS::CALLREQUEST")
  (make-package "XARM_MSGS::CALLREQUEST"))
(unless (find-package "XARM_MSGS::CALLRESPONSE")
  (make-package "XARM_MSGS::CALLRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::CallRequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::CallRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass xarm_msgs::CallResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::CallResponse
  (:init
   (&key
    ((:ret __ret) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _ret (round __ret))
   (setq _message (string __message))
   self)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; int16 _ret
    2
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::Call
  :super ros::object
  :slots ())

(setf (get xarm_msgs::Call :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::Call :datatype-) "xarm_msgs/Call")
(setf (get xarm_msgs::Call :request) xarm_msgs::CallRequest)
(setf (get xarm_msgs::Call :response) xarm_msgs::CallResponse)

(defmethod xarm_msgs::CallRequest
  (:response () (instance xarm_msgs::CallResponse :init)))

(setf (get xarm_msgs::CallRequest :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::CallRequest :datatype-) "xarm_msgs/CallRequest")
(setf (get xarm_msgs::CallRequest :definition-)
      "
---

int16 ret

string message
")

(setf (get xarm_msgs::CallResponse :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::CallResponse :datatype-) "xarm_msgs/CallResponse")
(setf (get xarm_msgs::CallResponse :definition-)
      "
---

int16 ret

string message
")



(provide :xarm_msgs/Call "76c68a2c5e109f4d6a4dc1cfc355f405")


