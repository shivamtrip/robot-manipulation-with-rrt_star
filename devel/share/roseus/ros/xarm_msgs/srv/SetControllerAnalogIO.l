;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetControllerAnalogIO)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetControllerAnalogIO (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETCONTROLLERANALOGIO")
  (make-package "XARM_MSGS::SETCONTROLLERANALOGIO"))
(unless (find-package "XARM_MSGS::SETCONTROLLERANALOGIOREQUEST")
  (make-package "XARM_MSGS::SETCONTROLLERANALOGIOREQUEST"))
(unless (find-package "XARM_MSGS::SETCONTROLLERANALOGIORESPONSE")
  (make-package "XARM_MSGS::SETCONTROLLERANALOGIORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetControllerAnalogIORequest
  :super ros::object
  :slots (_port_num _analog_value ))

(defmethod xarm_msgs::SetControllerAnalogIORequest
  (:init
   (&key
    ((:port_num __port_num) 0)
    ((:analog_value __analog_value) 0.0)
    )
   (send-super :init)
   (setq _port_num (round __port_num))
   (setq _analog_value (float __analog_value))
   self)
  (:port_num
   (&optional __port_num)
   (if __port_num (setq _port_num __port_num)) _port_num)
  (:analog_value
   (&optional __analog_value)
   (if __analog_value (setq _analog_value __analog_value)) _analog_value)
  (:serialization-length
   ()
   (+
    ;; int16 _port_num
    2
    ;; float32 _analog_value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _port_num
       (write-word _port_num s)
     ;; float32 _analog_value
       (sys::poke _analog_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _port_num
     (setq _port_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _analog_value
     (setq _analog_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::SetControllerAnalogIOResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetControllerAnalogIOResponse
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

(defclass xarm_msgs::SetControllerAnalogIO
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetControllerAnalogIO :md5sum-) "2b78c87698704b2bc72cea876ba90e55")
(setf (get xarm_msgs::SetControllerAnalogIO :datatype-) "xarm_msgs/SetControllerAnalogIO")
(setf (get xarm_msgs::SetControllerAnalogIO :request) xarm_msgs::SetControllerAnalogIORequest)
(setf (get xarm_msgs::SetControllerAnalogIO :response) xarm_msgs::SetControllerAnalogIOResponse)

(defmethod xarm_msgs::SetControllerAnalogIORequest
  (:response () (instance xarm_msgs::SetControllerAnalogIOResponse :init)))

(setf (get xarm_msgs::SetControllerAnalogIORequest :md5sum-) "2b78c87698704b2bc72cea876ba90e55")
(setf (get xarm_msgs::SetControllerAnalogIORequest :datatype-) "xarm_msgs/SetControllerAnalogIORequest")
(setf (get xarm_msgs::SetControllerAnalogIORequest :definition-)
      "# Setting the analog Output port value at robot controller, io_num: from 1 to 2

int16 port_num

float32 analog_value

---

int16 ret

string message

")

(setf (get xarm_msgs::SetControllerAnalogIOResponse :md5sum-) "2b78c87698704b2bc72cea876ba90e55")
(setf (get xarm_msgs::SetControllerAnalogIOResponse :datatype-) "xarm_msgs/SetControllerAnalogIOResponse")
(setf (get xarm_msgs::SetControllerAnalogIOResponse :definition-)
      "# Setting the analog Output port value at robot controller, io_num: from 1 to 2

int16 port_num

float32 analog_value

---

int16 ret

string message

")



(provide :xarm_msgs/SetControllerAnalogIO "2b78c87698704b2bc72cea876ba90e55")


