;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::IOState)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'IOState (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::IOSTATE")
  (make-package "XARM_MSGS::IOSTATE"))

(in-package "ROS")
;;//! \htmlinclude IOState.msg.html


(defclass xarm_msgs::IOState
  :super ros::object
  :slots (_digital_1 _digital_2 _analog_1 _analog_2 ))

(defmethod xarm_msgs::IOState
  (:init
   (&key
    ((:digital_1 __digital_1) 0)
    ((:digital_2 __digital_2) 0)
    ((:analog_1 __analog_1) 0.0)
    ((:analog_2 __analog_2) 0.0)
    )
   (send-super :init)
   (setq _digital_1 (round __digital_1))
   (setq _digital_2 (round __digital_2))
   (setq _analog_1 (float __analog_1))
   (setq _analog_2 (float __analog_2))
   self)
  (:digital_1
   (&optional __digital_1)
   (if __digital_1 (setq _digital_1 __digital_1)) _digital_1)
  (:digital_2
   (&optional __digital_2)
   (if __digital_2 (setq _digital_2 __digital_2)) _digital_2)
  (:analog_1
   (&optional __analog_1)
   (if __analog_1 (setq _analog_1 __analog_1)) _analog_1)
  (:analog_2
   (&optional __analog_2)
   (if __analog_2 (setq _analog_2 __analog_2)) _analog_2)
  (:serialization-length
   ()
   (+
    ;; int32 _digital_1
    4
    ;; int32 _digital_2
    4
    ;; float32 _analog_1
    4
    ;; float32 _analog_2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _digital_1
       (write-long _digital_1 s)
     ;; int32 _digital_2
       (write-long _digital_2 s)
     ;; float32 _analog_1
       (sys::poke _analog_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _analog_2
       (sys::poke _analog_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _digital_1
     (setq _digital_1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _digital_2
     (setq _digital_2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _analog_1
     (setq _analog_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _analog_2
     (setq _analog_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xarm_msgs::IOState :md5sum-) "cf35d5117b3f1964a89d007e947ecbee")
(setf (get xarm_msgs::IOState :datatype-) "xarm_msgs/IOState")
(setf (get xarm_msgs::IOState :definition-)
      "# for indicating 2 digital and 2 analog Input port state

int32 digital_1

int32 digital_2

float32 analog_1

float32 analog_2

")



(provide :xarm_msgs/IOState "cf35d5117b3f1964a89d007e947ecbee")


