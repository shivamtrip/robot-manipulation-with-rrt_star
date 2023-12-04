;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetFloat32)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetFloat32 (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETFLOAT32")
  (make-package "XARM_MSGS::SETFLOAT32"))
(unless (find-package "XARM_MSGS::SETFLOAT32REQUEST")
  (make-package "XARM_MSGS::SETFLOAT32REQUEST"))
(unless (find-package "XARM_MSGS::SETFLOAT32RESPONSE")
  (make-package "XARM_MSGS::SETFLOAT32RESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetFloat32Request
  :super ros::object
  :slots (_data ))

(defmethod xarm_msgs::SetFloat32Request
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::SetFloat32Response
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetFloat32Response
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

(defclass xarm_msgs::SetFloat32
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetFloat32 :md5sum-) "0eb9eaae8b6c9745e8550d0b1353340c")
(setf (get xarm_msgs::SetFloat32 :datatype-) "xarm_msgs/SetFloat32")
(setf (get xarm_msgs::SetFloat32 :request) xarm_msgs::SetFloat32Request)
(setf (get xarm_msgs::SetFloat32 :response) xarm_msgs::SetFloat32Response)

(defmethod xarm_msgs::SetFloat32Request
  (:response () (instance xarm_msgs::SetFloat32Response :init)))

(setf (get xarm_msgs::SetFloat32Request :md5sum-) "0eb9eaae8b6c9745e8550d0b1353340c")
(setf (get xarm_msgs::SetFloat32Request :datatype-) "xarm_msgs/SetFloat32Request")
(setf (get xarm_msgs::SetFloat32Request :definition-)
      "# request: set single floating point parameters.
# data: floating point (32-bit) parameter value.

float32 data

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")

(setf (get xarm_msgs::SetFloat32Response :md5sum-) "0eb9eaae8b6c9745e8550d0b1353340c")
(setf (get xarm_msgs::SetFloat32Response :datatype-) "xarm_msgs/SetFloat32Response")
(setf (get xarm_msgs::SetFloat32Response :definition-)
      "# request: set single floating point parameters.
# data: floating point (32-bit) parameter value.

float32 data

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")



(provide :xarm_msgs/SetFloat32 "0eb9eaae8b6c9745e8550d0b1353340c")


