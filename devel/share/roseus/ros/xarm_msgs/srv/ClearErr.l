;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::ClearErr)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'ClearErr (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::CLEARERR")
  (make-package "XARM_MSGS::CLEARERR"))
(unless (find-package "XARM_MSGS::CLEARERRREQUEST")
  (make-package "XARM_MSGS::CLEARERRREQUEST"))
(unless (find-package "XARM_MSGS::CLEARERRRESPONSE")
  (make-package "XARM_MSGS::CLEARERRRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::ClearErrRequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::ClearErrRequest
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

(defclass xarm_msgs::ClearErrResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::ClearErrResponse
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

(defclass xarm_msgs::ClearErr
  :super ros::object
  :slots ())

(setf (get xarm_msgs::ClearErr :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::ClearErr :datatype-) "xarm_msgs/ClearErr")
(setf (get xarm_msgs::ClearErr :request) xarm_msgs::ClearErrRequest)
(setf (get xarm_msgs::ClearErr :response) xarm_msgs::ClearErrResponse)

(defmethod xarm_msgs::ClearErrRequest
  (:response () (instance xarm_msgs::ClearErrResponse :init)))

(setf (get xarm_msgs::ClearErrRequest :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::ClearErrRequest :datatype-) "xarm_msgs/ClearErrRequest")
(setf (get xarm_msgs::ClearErrRequest :definition-)
      "
---

int16 ret

string message
")

(setf (get xarm_msgs::ClearErrResponse :md5sum-) "76c68a2c5e109f4d6a4dc1cfc355f405")
(setf (get xarm_msgs::ClearErrResponse :datatype-) "xarm_msgs/ClearErrResponse")
(setf (get xarm_msgs::ClearErrResponse :definition-)
      "
---

int16 ret

string message
")



(provide :xarm_msgs/ClearErr "76c68a2c5e109f4d6a4dc1cfc355f405")


