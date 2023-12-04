;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GripperConfig)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GripperConfig (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GRIPPERCONFIG")
  (make-package "XARM_MSGS::GRIPPERCONFIG"))
(unless (find-package "XARM_MSGS::GRIPPERCONFIGREQUEST")
  (make-package "XARM_MSGS::GRIPPERCONFIGREQUEST"))
(unless (find-package "XARM_MSGS::GRIPPERCONFIGRESPONSE")
  (make-package "XARM_MSGS::GRIPPERCONFIGRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GripperConfigRequest
  :super ros::object
  :slots (_pulse_vel ))

(defmethod xarm_msgs::GripperConfigRequest
  (:init
   (&key
    ((:pulse_vel __pulse_vel) 0.0)
    )
   (send-super :init)
   (setq _pulse_vel (float __pulse_vel))
   self)
  (:pulse_vel
   (&optional __pulse_vel)
   (if __pulse_vel (setq _pulse_vel __pulse_vel)) _pulse_vel)
  (:serialization-length
   ()
   (+
    ;; float32 _pulse_vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pulse_vel
       (sys::poke _pulse_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pulse_vel
     (setq _pulse_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::GripperConfigResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::GripperConfigResponse
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

(defclass xarm_msgs::GripperConfig
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GripperConfig :md5sum-) "52f0b064a7be31185f9e28fcaa5260ce")
(setf (get xarm_msgs::GripperConfig :datatype-) "xarm_msgs/GripperConfig")
(setf (get xarm_msgs::GripperConfig :request) xarm_msgs::GripperConfigRequest)
(setf (get xarm_msgs::GripperConfig :response) xarm_msgs::GripperConfigResponse)

(defmethod xarm_msgs::GripperConfigRequest
  (:response () (instance xarm_msgs::GripperConfigResponse :init)))

(setf (get xarm_msgs::GripperConfigRequest :md5sum-) "52f0b064a7be31185f9e28fcaa5260ce")
(setf (get xarm_msgs::GripperConfigRequest :datatype-) "xarm_msgs/GripperConfigRequest")
(setf (get xarm_msgs::GripperConfigRequest :definition-)
      "# Grip velocity configuration: range is from 1 to 5000
float32 pulse_vel

---

int16 ret
string message

")

(setf (get xarm_msgs::GripperConfigResponse :md5sum-) "52f0b064a7be31185f9e28fcaa5260ce")
(setf (get xarm_msgs::GripperConfigResponse :datatype-) "xarm_msgs/GripperConfigResponse")
(setf (get xarm_msgs::GripperConfigResponse :definition-)
      "# Grip velocity configuration: range is from 1 to 5000
float32 pulse_vel

---

int16 ret
string message

")



(provide :xarm_msgs/GripperConfig "52f0b064a7be31185f9e28fcaa5260ce")


