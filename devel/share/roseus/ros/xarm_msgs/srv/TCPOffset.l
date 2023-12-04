;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::TCPOffset)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'TCPOffset (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::TCPOFFSET")
  (make-package "XARM_MSGS::TCPOFFSET"))
(unless (find-package "XARM_MSGS::TCPOFFSETREQUEST")
  (make-package "XARM_MSGS::TCPOFFSETREQUEST"))
(unless (find-package "XARM_MSGS::TCPOFFSETRESPONSE")
  (make-package "XARM_MSGS::TCPOFFSETRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::TCPOffsetRequest
  :super ros::object
  :slots (_x _y _z _roll _pitch _yaw ))

(defmethod xarm_msgs::TCPOffsetRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::TCPOffsetResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::TCPOffsetResponse
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

(defclass xarm_msgs::TCPOffset
  :super ros::object
  :slots ())

(setf (get xarm_msgs::TCPOffset :md5sum-) "d56796195e02ffb01fe8b40cf84ab450")
(setf (get xarm_msgs::TCPOffset :datatype-) "xarm_msgs/TCPOffset")
(setf (get xarm_msgs::TCPOffset :request) xarm_msgs::TCPOffsetRequest)
(setf (get xarm_msgs::TCPOffset :response) xarm_msgs::TCPOffsetResponse)

(defmethod xarm_msgs::TCPOffsetRequest
  (:response () (instance xarm_msgs::TCPOffsetResponse :init)))

(setf (get xarm_msgs::TCPOffsetRequest :md5sum-) "d56796195e02ffb01fe8b40cf84ab450")
(setf (get xarm_msgs::TCPOffsetRequest :datatype-) "xarm_msgs/TCPOffsetRequest")
(setf (get xarm_msgs::TCPOffsetRequest :definition-)
      "# to give robot floating point number Cartesian TCP offset, based on initial Tool Frame located at flange center.

float32 x
float32 y
float32 z

float32 roll
float32 pitch
float32 yaw

---

int16 ret
string message

")

(setf (get xarm_msgs::TCPOffsetResponse :md5sum-) "d56796195e02ffb01fe8b40cf84ab450")
(setf (get xarm_msgs::TCPOffsetResponse :datatype-) "xarm_msgs/TCPOffsetResponse")
(setf (get xarm_msgs::TCPOffsetResponse :definition-)
      "# to give robot floating point number Cartesian TCP offset, based on initial Tool Frame located at flange center.

float32 x
float32 y
float32 z

float32 roll
float32 pitch
float32 yaw

---

int16 ret
string message

")



(provide :xarm_msgs/TCPOffset "d56796195e02ffb01fe8b40cf84ab450")


