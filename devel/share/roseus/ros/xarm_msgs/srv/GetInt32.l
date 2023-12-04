;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetInt32)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetInt32 (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETINT32")
  (make-package "XARM_MSGS::GETINT32"))
(unless (find-package "XARM_MSGS::GETINT32REQUEST")
  (make-package "XARM_MSGS::GETINT32REQUEST"))
(unless (find-package "XARM_MSGS::GETINT32RESPONSE")
  (make-package "XARM_MSGS::GETINT32RESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetInt32Request
  :super ros::object
  :slots ())

(defmethod xarm_msgs::GetInt32Request
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

(defclass xarm_msgs::GetInt32Response
  :super ros::object
  :slots (_data _ret _message ))

(defmethod xarm_msgs::GetInt32Response
  (:init
   (&key
    ((:data __data) 0)
    ((:ret __ret) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _data (round __data))
   (setq _ret (round __ret))
   (setq _message (string __message))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; int32 _data
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
     ;; int32 _data
       (write-long _data s)
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::GetInt32
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetInt32 :md5sum-) "32090468f6dc62f906d83554e3feecc6")
(setf (get xarm_msgs::GetInt32 :datatype-) "xarm_msgs/GetInt32")
(setf (get xarm_msgs::GetInt32 :request) xarm_msgs::GetInt32Request)
(setf (get xarm_msgs::GetInt32 :response) xarm_msgs::GetInt32Response)

(defmethod xarm_msgs::GetInt32Request
  (:response () (instance xarm_msgs::GetInt32Response :init)))

(setf (get xarm_msgs::GetInt32Request :md5sum-) "32090468f6dc62f906d83554e3feecc6")
(setf (get xarm_msgs::GetInt32Request :datatype-) "xarm_msgs/GetInt32Request")
(setf (get xarm_msgs::GetInt32Request :definition-)
      "
---

int32 data
int16 ret
string message
")

(setf (get xarm_msgs::GetInt32Response :md5sum-) "32090468f6dc62f906d83554e3feecc6")
(setf (get xarm_msgs::GetInt32Response :datatype-) "xarm_msgs/GetInt32Response")
(setf (get xarm_msgs::GetInt32Response :definition-)
      "
---

int32 data
int16 ret
string message
")



(provide :xarm_msgs/GetInt32 "32090468f6dc62f906d83554e3feecc6")


