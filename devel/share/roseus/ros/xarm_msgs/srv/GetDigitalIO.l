;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetDigitalIO)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetDigitalIO (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETDIGITALIO")
  (make-package "XARM_MSGS::GETDIGITALIO"))
(unless (find-package "XARM_MSGS::GETDIGITALIOREQUEST")
  (make-package "XARM_MSGS::GETDIGITALIOREQUEST"))
(unless (find-package "XARM_MSGS::GETDIGITALIORESPONSE")
  (make-package "XARM_MSGS::GETDIGITALIORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetDigitalIORequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::GetDigitalIORequest
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

(defclass xarm_msgs::GetDigitalIOResponse
  :super ros::object
  :slots (_digital_1 _digital_2 _ret _message ))

(defmethod xarm_msgs::GetDigitalIOResponse
  (:init
   (&key
    ((:digital_1 __digital_1) 0)
    ((:digital_2 __digital_2) 0)
    ((:ret __ret) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _digital_1 (round __digital_1))
   (setq _digital_2 (round __digital_2))
   (setq _ret (round __ret))
   (setq _message (string __message))
   self)
  (:digital_1
   (&optional __digital_1)
   (if __digital_1 (setq _digital_1 __digital_1)) _digital_1)
  (:digital_2
   (&optional __digital_2)
   (if __digital_2 (setq _digital_2 __digital_2)) _digital_2)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; int32 _digital_1
    4
    ;; int32 _digital_2
    4
    ;; int16 _ret
    2
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _digital_1
       (write-long _digital_1 s)
     ;; int32 _digital_2
       (write-long _digital_2 s)
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _digital_1
     (setq _digital_1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _digital_2
     (setq _digital_2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::GetDigitalIO
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetDigitalIO :md5sum-) "5c2bfd923228c71a217e97a1d1747b99")
(setf (get xarm_msgs::GetDigitalIO :datatype-) "xarm_msgs/GetDigitalIO")
(setf (get xarm_msgs::GetDigitalIO :request) xarm_msgs::GetDigitalIORequest)
(setf (get xarm_msgs::GetDigitalIO :response) xarm_msgs::GetDigitalIOResponse)

(defmethod xarm_msgs::GetDigitalIORequest
  (:response () (instance xarm_msgs::GetDigitalIOResponse :init)))

(setf (get xarm_msgs::GetDigitalIORequest :md5sum-) "5c2bfd923228c71a217e97a1d1747b99")
(setf (get xarm_msgs::GetDigitalIORequest :datatype-) "xarm_msgs/GetDigitalIORequest")
(setf (get xarm_msgs::GetDigitalIORequest :definition-)
      "# Getting the 2 digital Input port at robot end connector

---

int32 digital_1

int32 digital_2

int16 ret

string message
")

(setf (get xarm_msgs::GetDigitalIOResponse :md5sum-) "5c2bfd923228c71a217e97a1d1747b99")
(setf (get xarm_msgs::GetDigitalIOResponse :datatype-) "xarm_msgs/GetDigitalIOResponse")
(setf (get xarm_msgs::GetDigitalIOResponse :definition-)
      "# Getting the 2 digital Input port at robot end connector

---

int32 digital_1

int32 digital_2

int16 ret

string message
")



(provide :xarm_msgs/GetDigitalIO "5c2bfd923228c71a217e97a1d1747b99")


