;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GripperMove)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GripperMove (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GRIPPERMOVE")
  (make-package "XARM_MSGS::GRIPPERMOVE"))
(unless (find-package "XARM_MSGS::GRIPPERMOVEREQUEST")
  (make-package "XARM_MSGS::GRIPPERMOVEREQUEST"))
(unless (find-package "XARM_MSGS::GRIPPERMOVERESPONSE")
  (make-package "XARM_MSGS::GRIPPERMOVERESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GripperMoveRequest
  :super ros::object
  :slots (_pulse_pos ))

(defmethod xarm_msgs::GripperMoveRequest
  (:init
   (&key
    ((:pulse_pos __pulse_pos) 0.0)
    )
   (send-super :init)
   (setq _pulse_pos (float __pulse_pos))
   self)
  (:pulse_pos
   (&optional __pulse_pos)
   (if __pulse_pos (setq _pulse_pos __pulse_pos)) _pulse_pos)
  (:serialization-length
   ()
   (+
    ;; float32 _pulse_pos
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pulse_pos
       (sys::poke _pulse_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pulse_pos
     (setq _pulse_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::GripperMoveResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::GripperMoveResponse
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

(defclass xarm_msgs::GripperMove
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GripperMove :md5sum-) "68b32f775972a0898b388d8b66a9b5d1")
(setf (get xarm_msgs::GripperMove :datatype-) "xarm_msgs/GripperMove")
(setf (get xarm_msgs::GripperMove :request) xarm_msgs::GripperMoveRequest)
(setf (get xarm_msgs::GripperMove :response) xarm_msgs::GripperMoveResponse)

(defmethod xarm_msgs::GripperMoveRequest
  (:response () (instance xarm_msgs::GripperMoveResponse :init)))

(setf (get xarm_msgs::GripperMoveRequest :md5sum-) "68b32f775972a0898b388d8b66a9b5d1")
(setf (get xarm_msgs::GripperMoveRequest :datatype-) "xarm_msgs/GripperMoveRequest")
(setf (get xarm_msgs::GripperMoveRequest :definition-)
      "# position command of gripper: range from 0(close) to 850 (open)
float32 pulse_pos

---

int16 ret
string message
")

(setf (get xarm_msgs::GripperMoveResponse :md5sum-) "68b32f775972a0898b388d8b66a9b5d1")
(setf (get xarm_msgs::GripperMoveResponse :datatype-) "xarm_msgs/GripperMoveResponse")
(setf (get xarm_msgs::GripperMoveResponse :definition-)
      "# position command of gripper: range from 0(close) to 850 (open)
float32 pulse_pos

---

int16 ret
string message
")



(provide :xarm_msgs/GripperMove "68b32f775972a0898b388d8b66a9b5d1")


