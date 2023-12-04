;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetControllerDigitalIO)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetControllerDigitalIO (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETCONTROLLERDIGITALIO")
  (make-package "XARM_MSGS::GETCONTROLLERDIGITALIO"))
(unless (find-package "XARM_MSGS::GETCONTROLLERDIGITALIOREQUEST")
  (make-package "XARM_MSGS::GETCONTROLLERDIGITALIOREQUEST"))
(unless (find-package "XARM_MSGS::GETCONTROLLERDIGITALIORESPONSE")
  (make-package "XARM_MSGS::GETCONTROLLERDIGITALIORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetControllerDigitalIORequest
  :super ros::object
  :slots (_io_num ))

(defmethod xarm_msgs::GetControllerDigitalIORequest
  (:init
   (&key
    ((:io_num __io_num) 0)
    )
   (send-super :init)
   (setq _io_num (round __io_num))
   self)
  (:io_num
   (&optional __io_num)
   (if __io_num (setq _io_num __io_num)) _io_num)
  (:serialization-length
   ()
   (+
    ;; int16 _io_num
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _io_num
       (write-word _io_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _io_num
     (setq _io_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::GetControllerDigitalIOResponse
  :super ros::object
  :slots (_ret _value _message ))

(defmethod xarm_msgs::GetControllerDigitalIOResponse
  (:init
   (&key
    ((:ret __ret) 0)
    ((:value __value) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _ret (round __ret))
   (setq _value (round __value))
   (setq _message (string __message))
   self)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; int16 _ret
    2
    ;; int16 _value
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
     ;; int16 _value
       (write-word _value s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::GetControllerDigitalIO
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetControllerDigitalIO :md5sum-) "a10e26b4b2c853a8b561f9587bff33b2")
(setf (get xarm_msgs::GetControllerDigitalIO :datatype-) "xarm_msgs/GetControllerDigitalIO")
(setf (get xarm_msgs::GetControllerDigitalIO :request) xarm_msgs::GetControllerDigitalIORequest)
(setf (get xarm_msgs::GetControllerDigitalIO :response) xarm_msgs::GetControllerDigitalIOResponse)

(defmethod xarm_msgs::GetControllerDigitalIORequest
  (:response () (instance xarm_msgs::GetControllerDigitalIOResponse :init)))

(setf (get xarm_msgs::GetControllerDigitalIORequest :md5sum-) "a10e26b4b2c853a8b561f9587bff33b2")
(setf (get xarm_msgs::GetControllerDigitalIORequest :datatype-) "xarm_msgs/GetControllerDigitalIORequest")
(setf (get xarm_msgs::GetControllerDigitalIORequest :definition-)
      "# Getting the controller DIGITAL input port status, io_num: from 1 to 16

int16 io_num

---

int16 ret

int16 value

string message

")

(setf (get xarm_msgs::GetControllerDigitalIOResponse :md5sum-) "a10e26b4b2c853a8b561f9587bff33b2")
(setf (get xarm_msgs::GetControllerDigitalIOResponse :datatype-) "xarm_msgs/GetControllerDigitalIOResponse")
(setf (get xarm_msgs::GetControllerDigitalIOResponse :definition-)
      "# Getting the controller DIGITAL input port status, io_num: from 1 to 16

int16 io_num

---

int16 ret

int16 value

string message

")



(provide :xarm_msgs/GetControllerDigitalIO "a10e26b4b2c853a8b561f9587bff33b2")


