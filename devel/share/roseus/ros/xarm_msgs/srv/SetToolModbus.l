;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetToolModbus)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetToolModbus (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETTOOLMODBUS")
  (make-package "XARM_MSGS::SETTOOLMODBUS"))
(unless (find-package "XARM_MSGS::SETTOOLMODBUSREQUEST")
  (make-package "XARM_MSGS::SETTOOLMODBUSREQUEST"))
(unless (find-package "XARM_MSGS::SETTOOLMODBUSRESPONSE")
  (make-package "XARM_MSGS::SETTOOLMODBUSRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetToolModbusRequest
  :super ros::object
  :slots (_send_data _respond_len ))

(defmethod xarm_msgs::SetToolModbusRequest
  (:init
   (&key
    ((:send_data __send_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:respond_len __respond_len) 0)
    )
   (send-super :init)
   (setq _send_data __send_data)
   (setq _respond_len (round __respond_len))
   self)
  (:send_data
   (&optional __send_data)
   (if __send_data (setq _send_data __send_data)) _send_data)
  (:respond_len
   (&optional __respond_len)
   (if __respond_len (setq _respond_len __respond_len)) _respond_len)
  (:serialization-length
   ()
   (+
    ;; uint8[] _send_data
    (* 1    (length _send_data)) 4
    ;; int16 _respond_len
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _send_data
     (write-long (length _send_data) s)
     (princ _send_data s)
     ;; int16 _respond_len
       (write-word _respond_len s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _send_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _send_data (make-array n :element-type :char))
     (replace _send_data buf :start2 ptr-) (incf ptr- n))
   ;; int16 _respond_len
     (setq _respond_len (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::SetToolModbusResponse
  :super ros::object
  :slots (_ret _message _respond_data ))

(defmethod xarm_msgs::SetToolModbusResponse
  (:init
   (&key
    ((:ret __ret) 0)
    ((:message __message) "")
    ((:respond_data __respond_data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _ret (round __ret))
   (setq _message (string __message))
   (setq _respond_data __respond_data)
   self)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:respond_data
   (&optional __respond_data)
   (if __respond_data (setq _respond_data __respond_data)) _respond_data)
  (:serialization-length
   ()
   (+
    ;; int16 _ret
    2
    ;; string _message
    4 (length _message)
    ;; uint8[] _respond_data
    (* 1    (length _respond_data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; uint8[] _respond_data
     (write-long (length _respond_data) s)
     (princ _respond_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8[] _respond_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _respond_data (make-array n :element-type :char))
     (replace _respond_data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(defclass xarm_msgs::SetToolModbus
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetToolModbus :md5sum-) "0b8299eb42a5262c78587a39fca90c6d")
(setf (get xarm_msgs::SetToolModbus :datatype-) "xarm_msgs/SetToolModbus")
(setf (get xarm_msgs::SetToolModbus :request) xarm_msgs::SetToolModbusRequest)
(setf (get xarm_msgs::SetToolModbus :response) xarm_msgs::SetToolModbusResponse)

(defmethod xarm_msgs::SetToolModbusRequest
  (:response () (instance xarm_msgs::SetToolModbusResponse :init)))

(setf (get xarm_msgs::SetToolModbusRequest :md5sum-) "0b8299eb42a5262c78587a39fca90c6d")
(setf (get xarm_msgs::SetToolModbusRequest :datatype-) "xarm_msgs/SetToolModbusRequest")
(setf (get xarm_msgs::SetToolModbusRequest :definition-)
      "# unsigned char data to be sent to tool device through modbus
uint8[] send_data

# Specify the anticipated maximum respond data length in bytes
int16 respond_len

---

int16 ret
string message
uint8[] respond_data
")

(setf (get xarm_msgs::SetToolModbusResponse :md5sum-) "0b8299eb42a5262c78587a39fca90c6d")
(setf (get xarm_msgs::SetToolModbusResponse :datatype-) "xarm_msgs/SetToolModbusResponse")
(setf (get xarm_msgs::SetToolModbusResponse :definition-)
      "# unsigned char data to be sent to tool device through modbus
uint8[] send_data

# Specify the anticipated maximum respond data length in bytes
int16 respond_len

---

int16 ret
string message
uint8[] respond_data
")



(provide :xarm_msgs/SetToolModbus "0b8299eb42a5262c78587a39fca90c6d")


