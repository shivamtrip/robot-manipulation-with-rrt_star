;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetInt16)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetInt16 (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETINT16")
  (make-package "XARM_MSGS::SETINT16"))
(unless (find-package "XARM_MSGS::SETINT16REQUEST")
  (make-package "XARM_MSGS::SETINT16REQUEST"))
(unless (find-package "XARM_MSGS::SETINT16RESPONSE")
  (make-package "XARM_MSGS::SETINT16RESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetInt16Request
  :super ros::object
  :slots (_data ))

(defmethod xarm_msgs::SetInt16Request
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::SetInt16Response
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetInt16Response
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

(defclass xarm_msgs::SetInt16
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetInt16 :md5sum-) "b107d9652f6d76aa4c20bfd2c517f4f8")
(setf (get xarm_msgs::SetInt16 :datatype-) "xarm_msgs/SetInt16")
(setf (get xarm_msgs::SetInt16 :request) xarm_msgs::SetInt16Request)
(setf (get xarm_msgs::SetInt16 :response) xarm_msgs::SetInt16Response)

(defmethod xarm_msgs::SetInt16Request
  (:response () (instance xarm_msgs::SetInt16Response :init)))

(setf (get xarm_msgs::SetInt16Request :md5sum-) "b107d9652f6d76aa4c20bfd2c517f4f8")
(setf (get xarm_msgs::SetInt16Request :datatype-) "xarm_msgs/SetInt16Request")
(setf (get xarm_msgs::SetInt16Request :definition-)
      "# request: set robot state or mode.
# data: int value indicating command robot state or execution mode.

int16 data

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message

")

(setf (get xarm_msgs::SetInt16Response :md5sum-) "b107d9652f6d76aa4c20bfd2c517f4f8")
(setf (get xarm_msgs::SetInt16Response :datatype-) "xarm_msgs/SetInt16Response")
(setf (get xarm_msgs::SetInt16Response :definition-)
      "# request: set robot state or mode.
# data: int value indicating command robot state or execution mode.

int16 data

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message

")



(provide :xarm_msgs/SetInt16 "b107d9652f6d76aa4c20bfd2c517f4f8")


