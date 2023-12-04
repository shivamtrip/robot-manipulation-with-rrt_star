;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GripperState)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GripperState (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GRIPPERSTATE")
  (make-package "XARM_MSGS::GRIPPERSTATE"))
(unless (find-package "XARM_MSGS::GRIPPERSTATEREQUEST")
  (make-package "XARM_MSGS::GRIPPERSTATEREQUEST"))
(unless (find-package "XARM_MSGS::GRIPPERSTATERESPONSE")
  (make-package "XARM_MSGS::GRIPPERSTATERESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GripperStateRequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::GripperStateRequest
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

(defclass xarm_msgs::GripperStateResponse
  :super ros::object
  :slots (_ret _message _curr_pos _err_code ))

(defmethod xarm_msgs::GripperStateResponse
  (:init
   (&key
    ((:ret __ret) 0)
    ((:message __message) "")
    ((:curr_pos __curr_pos) 0.0)
    ((:err_code __err_code) 0)
    )
   (send-super :init)
   (setq _ret (round __ret))
   (setq _message (string __message))
   (setq _curr_pos (float __curr_pos))
   (setq _err_code (round __err_code))
   self)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:curr_pos
   (&optional __curr_pos)
   (if __curr_pos (setq _curr_pos __curr_pos)) _curr_pos)
  (:err_code
   (&optional __err_code)
   (if __err_code (setq _err_code __err_code)) _err_code)
  (:serialization-length
   ()
   (+
    ;; int16 _ret
    2
    ;; string _message
    4 (length _message)
    ;; float32 _curr_pos
    4
    ;; int16 _err_code
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _curr_pos
       (sys::poke _curr_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _err_code
       (write-word _err_code s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _curr_pos
     (setq _curr_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _err_code
     (setq _err_code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::GripperState
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GripperState :md5sum-) "b5eb0261d03e545bc9905bb8e7e041a8")
(setf (get xarm_msgs::GripperState :datatype-) "xarm_msgs/GripperState")
(setf (get xarm_msgs::GripperState :request) xarm_msgs::GripperStateRequest)
(setf (get xarm_msgs::GripperState :response) xarm_msgs::GripperStateResponse)

(defmethod xarm_msgs::GripperStateRequest
  (:response () (instance xarm_msgs::GripperStateResponse :init)))

(setf (get xarm_msgs::GripperStateRequest :md5sum-) "b5eb0261d03e545bc9905bb8e7e041a8")
(setf (get xarm_msgs::GripperStateRequest :datatype-) "xarm_msgs/GripperStateRequest")
(setf (get xarm_msgs::GripperStateRequest :definition-)
      "---
int16 ret
string message
float32 curr_pos
int16 err_code
")

(setf (get xarm_msgs::GripperStateResponse :md5sum-) "b5eb0261d03e545bc9905bb8e7e041a8")
(setf (get xarm_msgs::GripperStateResponse :datatype-) "xarm_msgs/GripperStateResponse")
(setf (get xarm_msgs::GripperStateResponse :definition-)
      "---
int16 ret
string message
float32 curr_pos
int16 err_code
")



(provide :xarm_msgs/GripperState "b5eb0261d03e545bc9905bb8e7e041a8")


