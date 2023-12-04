;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::ConfigToolModbus)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'ConfigToolModbus (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::CONFIGTOOLMODBUS")
  (make-package "XARM_MSGS::CONFIGTOOLMODBUS"))
(unless (find-package "XARM_MSGS::CONFIGTOOLMODBUSREQUEST")
  (make-package "XARM_MSGS::CONFIGTOOLMODBUSREQUEST"))
(unless (find-package "XARM_MSGS::CONFIGTOOLMODBUSRESPONSE")
  (make-package "XARM_MSGS::CONFIGTOOLMODBUSRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::ConfigToolModbusRequest
  :super ros::object
  :slots (_baud_rate _timeout_ms ))

(defmethod xarm_msgs::ConfigToolModbusRequest
  (:init
   (&key
    ((:baud_rate __baud_rate) 0)
    ((:timeout_ms __timeout_ms) 0)
    )
   (send-super :init)
   (setq _baud_rate (round __baud_rate))
   (setq _timeout_ms (round __timeout_ms))
   self)
  (:baud_rate
   (&optional __baud_rate)
   (if __baud_rate (setq _baud_rate __baud_rate)) _baud_rate)
  (:timeout_ms
   (&optional __timeout_ms)
   (if __timeout_ms (setq _timeout_ms __timeout_ms)) _timeout_ms)
  (:serialization-length
   ()
   (+
    ;; int32 _baud_rate
    4
    ;; int32 _timeout_ms
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _baud_rate
       (write-long _baud_rate s)
     ;; int32 _timeout_ms
       (write-long _timeout_ms s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _baud_rate
     (setq _baud_rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _timeout_ms
     (setq _timeout_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::ConfigToolModbusResponse
  :super ros::object
  :slots (_message _ret ))

(defmethod xarm_msgs::ConfigToolModbusResponse
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

(defclass xarm_msgs::ConfigToolModbus
  :super ros::object
  :slots ())

(setf (get xarm_msgs::ConfigToolModbus :md5sum-) "4641743544bd81148c56355ba50062be")
(setf (get xarm_msgs::ConfigToolModbus :datatype-) "xarm_msgs/ConfigToolModbus")
(setf (get xarm_msgs::ConfigToolModbus :request) xarm_msgs::ConfigToolModbusRequest)
(setf (get xarm_msgs::ConfigToolModbus :response) xarm_msgs::ConfigToolModbusResponse)

(defmethod xarm_msgs::ConfigToolModbusRequest
  (:response () (instance xarm_msgs::ConfigToolModbusResponse :init)))

(setf (get xarm_msgs::ConfigToolModbusRequest :md5sum-) "4641743544bd81148c56355ba50062be")
(setf (get xarm_msgs::ConfigToolModbusRequest :datatype-) "xarm_msgs/ConfigToolModbusRequest")
(setf (get xarm_msgs::ConfigToolModbusRequest :definition-)
      "# configure the tool modbus communication baud rate, in bps:
int32 baud_rate

# configure the timeout parameter in modbus communication, in milliseconds
int32 timeout_ms

---

string message
int16 ret
")

(setf (get xarm_msgs::ConfigToolModbusResponse :md5sum-) "4641743544bd81148c56355ba50062be")
(setf (get xarm_msgs::ConfigToolModbusResponse :datatype-) "xarm_msgs/ConfigToolModbusResponse")
(setf (get xarm_msgs::ConfigToolModbusResponse :definition-)
      "# configure the tool modbus communication baud rate, in bps:
int32 baud_rate

# configure the timeout parameter in modbus communication, in milliseconds
int32 timeout_ms

---

string message
int16 ret
")



(provide :xarm_msgs/ConfigToolModbus "4641743544bd81148c56355ba50062be")


