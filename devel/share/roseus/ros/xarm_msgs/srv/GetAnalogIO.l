;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetAnalogIO)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetAnalogIO (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETANALOGIO")
  (make-package "XARM_MSGS::GETANALOGIO"))
(unless (find-package "XARM_MSGS::GETANALOGIOREQUEST")
  (make-package "XARM_MSGS::GETANALOGIOREQUEST"))
(unless (find-package "XARM_MSGS::GETANALOGIORESPONSE")
  (make-package "XARM_MSGS::GETANALOGIORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetAnalogIORequest
  :super ros::object
  :slots (_port_num ))

(defmethod xarm_msgs::GetAnalogIORequest
  (:init
   (&key
    ((:port_num __port_num) 0)
    )
   (send-super :init)
   (setq _port_num (round __port_num))
   self)
  (:port_num
   (&optional __port_num)
   (if __port_num (setq _port_num __port_num)) _port_num)
  (:serialization-length
   ()
   (+
    ;; int16 _port_num
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _port_num
       (write-word _port_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _port_num
     (setq _port_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::GetAnalogIOResponse
  :super ros::object
  :slots (_analog_value _ret _message ))

(defmethod xarm_msgs::GetAnalogIOResponse
  (:init
   (&key
    ((:analog_value __analog_value) 0.0)
    ((:ret __ret) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _analog_value (float __analog_value))
   (setq _ret (round __ret))
   (setq _message (string __message))
   self)
  (:analog_value
   (&optional __analog_value)
   (if __analog_value (setq _analog_value __analog_value)) _analog_value)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; float32 _analog_value
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
     ;; float32 _analog_value
       (sys::poke _analog_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _analog_value
     (setq _analog_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::GetAnalogIO
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetAnalogIO :md5sum-) "be8d9a2c0ed50c727cbf098654568f97")
(setf (get xarm_msgs::GetAnalogIO :datatype-) "xarm_msgs/GetAnalogIO")
(setf (get xarm_msgs::GetAnalogIO :request) xarm_msgs::GetAnalogIORequest)
(setf (get xarm_msgs::GetAnalogIO :response) xarm_msgs::GetAnalogIOResponse)

(defmethod xarm_msgs::GetAnalogIORequest
  (:response () (instance xarm_msgs::GetAnalogIOResponse :init)))

(setf (get xarm_msgs::GetAnalogIORequest :md5sum-) "be8d9a2c0ed50c727cbf098654568f97")
(setf (get xarm_msgs::GetAnalogIORequest :datatype-) "xarm_msgs/GetAnalogIORequest")
(setf (get xarm_msgs::GetAnalogIORequest :definition-)
      "# Getting one of the 2 analog Input port at robot end connector or controller

int16 port_num

---

float32 analog_value

int16 ret

string message

")

(setf (get xarm_msgs::GetAnalogIOResponse :md5sum-) "be8d9a2c0ed50c727cbf098654568f97")
(setf (get xarm_msgs::GetAnalogIOResponse :datatype-) "xarm_msgs/GetAnalogIOResponse")
(setf (get xarm_msgs::GetAnalogIOResponse :definition-)
      "# Getting one of the 2 analog Input port at robot end connector or controller

int16 port_num

---

float32 analog_value

int16 ret

string message

")



(provide :xarm_msgs/GetAnalogIO "be8d9a2c0ed50c727cbf098654568f97")


