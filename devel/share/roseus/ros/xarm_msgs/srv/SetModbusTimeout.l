;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetModbusTimeout)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetModbusTimeout (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETMODBUSTIMEOUT")
  (make-package "XARM_MSGS::SETMODBUSTIMEOUT"))
(unless (find-package "XARM_MSGS::SETMODBUSTIMEOUTREQUEST")
  (make-package "XARM_MSGS::SETMODBUSTIMEOUTREQUEST"))
(unless (find-package "XARM_MSGS::SETMODBUSTIMEOUTRESPONSE")
  (make-package "XARM_MSGS::SETMODBUSTIMEOUTRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetModbusTimeoutRequest
  :super ros::object
  :slots (_timeout_ms _is_transparent_transmission ))

(defmethod xarm_msgs::SetModbusTimeoutRequest
  (:init
   (&key
    ((:timeout_ms __timeout_ms) 0)
    ((:is_transparent_transmission __is_transparent_transmission) nil)
    )
   (send-super :init)
   (setq _timeout_ms (round __timeout_ms))
   (setq _is_transparent_transmission __is_transparent_transmission)
   self)
  (:timeout_ms
   (&optional __timeout_ms)
   (if __timeout_ms (setq _timeout_ms __timeout_ms)) _timeout_ms)
  (:is_transparent_transmission
   (&optional (__is_transparent_transmission :null))
   (if (not (eq __is_transparent_transmission :null)) (setq _is_transparent_transmission __is_transparent_transmission)) _is_transparent_transmission)
  (:serialization-length
   ()
   (+
    ;; int32 _timeout_ms
    4
    ;; bool _is_transparent_transmission
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _timeout_ms
       (write-long _timeout_ms s)
     ;; bool _is_transparent_transmission
       (if _is_transparent_transmission (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _timeout_ms
     (setq _timeout_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_transparent_transmission
     (setq _is_transparent_transmission (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_msgs::SetModbusTimeoutResponse
  :super ros::object
  :slots (_message _ret ))

(defmethod xarm_msgs::SetModbusTimeoutResponse
  (:init
   (&key
    ((:message __message) "")
    ((:ret __ret) 0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _ret (round __ret))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; int16 _ret
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; int16 _ret
       (write-word _ret s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::SetModbusTimeout
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetModbusTimeout :md5sum-) "cd53862105e4494b607dc1eaae204c5f")
(setf (get xarm_msgs::SetModbusTimeout :datatype-) "xarm_msgs/SetModbusTimeout")
(setf (get xarm_msgs::SetModbusTimeout :request) xarm_msgs::SetModbusTimeoutRequest)
(setf (get xarm_msgs::SetModbusTimeout :response) xarm_msgs::SetModbusTimeoutResponse)

(defmethod xarm_msgs::SetModbusTimeoutRequest
  (:response () (instance xarm_msgs::SetModbusTimeoutResponse :init)))

(setf (get xarm_msgs::SetModbusTimeoutRequest :md5sum-) "cd53862105e4494b607dc1eaae204c5f")
(setf (get xarm_msgs::SetModbusTimeoutRequest :datatype-) "xarm_msgs/SetModbusTimeoutRequest")
(setf (get xarm_msgs::SetModbusTimeoutRequest :definition-)
      "# configure the timeout parameter in modbus communication, in milliseconds
int32 timeout_ms

# whether the set timeout is the timeout of transparent transmission
bool is_transparent_transmission

---

string message
int16 ret
")

(setf (get xarm_msgs::SetModbusTimeoutResponse :md5sum-) "cd53862105e4494b607dc1eaae204c5f")
(setf (get xarm_msgs::SetModbusTimeoutResponse :datatype-) "xarm_msgs/SetModbusTimeoutResponse")
(setf (get xarm_msgs::SetModbusTimeoutResponse :definition-)
      "# configure the timeout parameter in modbus communication, in milliseconds
int32 timeout_ms

# whether the set timeout is the timeout of transparent transmission
bool is_transparent_transmission

---

string message
int16 ret
")



(provide :xarm_msgs/SetModbusTimeout "cd53862105e4494b607dc1eaae204c5f")


