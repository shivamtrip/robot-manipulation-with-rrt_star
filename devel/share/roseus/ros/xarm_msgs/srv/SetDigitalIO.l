;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetDigitalIO)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetDigitalIO (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETDIGITALIO")
  (make-package "XARM_MSGS::SETDIGITALIO"))
(unless (find-package "XARM_MSGS::SETDIGITALIOREQUEST")
  (make-package "XARM_MSGS::SETDIGITALIOREQUEST"))
(unless (find-package "XARM_MSGS::SETDIGITALIORESPONSE")
  (make-package "XARM_MSGS::SETDIGITALIORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetDigitalIORequest
  :super ros::object
  :slots (_io_num _value ))

(defmethod xarm_msgs::SetDigitalIORequest
  (:init
   (&key
    ((:io_num __io_num) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _io_num (round __io_num))
   (setq _value (round __value))
   self)
  (:io_num
   (&optional __io_num)
   (if __io_num (setq _io_num __io_num)) _io_num)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int16 _io_num
    2
    ;; int16 _value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _io_num
       (write-word _io_num s)
     ;; int16 _value
       (write-word _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _io_num
     (setq _io_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::SetDigitalIOResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetDigitalIOResponse
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

(defclass xarm_msgs::SetDigitalIO
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetDigitalIO :md5sum-) "50ac3ff571bfc64bb298f9570217847f")
(setf (get xarm_msgs::SetDigitalIO :datatype-) "xarm_msgs/SetDigitalIO")
(setf (get xarm_msgs::SetDigitalIO :request) xarm_msgs::SetDigitalIORequest)
(setf (get xarm_msgs::SetDigitalIO :response) xarm_msgs::SetDigitalIOResponse)

(defmethod xarm_msgs::SetDigitalIORequest
  (:response () (instance xarm_msgs::SetDigitalIOResponse :init)))

(setf (get xarm_msgs::SetDigitalIORequest :md5sum-) "50ac3ff571bfc64bb298f9570217847f")
(setf (get xarm_msgs::SetDigitalIORequest :datatype-) "xarm_msgs/SetDigitalIORequest")
(setf (get xarm_msgs::SetDigitalIORequest :definition-)
      "# Setting the digital Output port at control box or robot end connector

int16 io_num

int16 value

---

int16 ret

string message
")

(setf (get xarm_msgs::SetDigitalIOResponse :md5sum-) "50ac3ff571bfc64bb298f9570217847f")
(setf (get xarm_msgs::SetDigitalIOResponse :datatype-) "xarm_msgs/SetDigitalIOResponse")
(setf (get xarm_msgs::SetDigitalIOResponse :definition-)
      "# Setting the digital Output port at control box or robot end connector

int16 io_num

int16 value

---

int16 ret

string message
")



(provide :xarm_msgs/SetDigitalIO "50ac3ff571bfc64bb298f9570217847f")


