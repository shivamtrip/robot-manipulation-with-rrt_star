;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetSetModbusData)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetSetModbusData (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETSETMODBUSDATA")
  (make-package "XARM_MSGS::GETSETMODBUSDATA"))
(unless (find-package "XARM_MSGS::GETSETMODBUSDATAREQUEST")
  (make-package "XARM_MSGS::GETSETMODBUSDATAREQUEST"))
(unless (find-package "XARM_MSGS::GETSETMODBUSDATARESPONSE")
  (make-package "XARM_MSGS::GETSETMODBUSDATARESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetSetModbusDataRequest
  :super ros::object
  :slots (_send_data _respond_len _host_id _is_transparent_transmission _use_503_port ))

(defmethod xarm_msgs::GetSetModbusDataRequest
  (:init
   (&key
    ((:send_data __send_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:respond_len __respond_len) 0)
    ((:host_id __host_id) 0)
    ((:is_transparent_transmission __is_transparent_transmission) nil)
    ((:use_503_port __use_503_port) nil)
    )
   (send-super :init)
   (setq _send_data __send_data)
   (setq _respond_len (round __respond_len))
   (setq _host_id (round __host_id))
   (setq _is_transparent_transmission __is_transparent_transmission)
   (setq _use_503_port __use_503_port)
   self)
  (:send_data
   (&optional __send_data)
   (if __send_data (setq _send_data __send_data)) _send_data)
  (:respond_len
   (&optional __respond_len)
   (if __respond_len (setq _respond_len __respond_len)) _respond_len)
  (:host_id
   (&optional __host_id)
   (if __host_id (setq _host_id __host_id)) _host_id)
  (:is_transparent_transmission
   (&optional (__is_transparent_transmission :null))
   (if (not (eq __is_transparent_transmission :null)) (setq _is_transparent_transmission __is_transparent_transmission)) _is_transparent_transmission)
  (:use_503_port
   (&optional (__use_503_port :null))
   (if (not (eq __use_503_port :null)) (setq _use_503_port __use_503_port)) _use_503_port)
  (:serialization-length
   ()
   (+
    ;; uint8[] _send_data
    (* 1    (length _send_data)) 4
    ;; int16 _respond_len
    2
    ;; uint8 _host_id
    1
    ;; bool _is_transparent_transmission
    1
    ;; bool _use_503_port
    1
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
     ;; uint8 _host_id
       (write-byte _host_id s)
     ;; bool _is_transparent_transmission
       (if _is_transparent_transmission (write-byte -1 s) (write-byte 0 s))
     ;; bool _use_503_port
       (if _use_503_port (write-byte -1 s) (write-byte 0 s))
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
   ;; uint8 _host_id
     (setq _host_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _is_transparent_transmission
     (setq _is_transparent_transmission (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _use_503_port
     (setq _use_503_port (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_msgs::GetSetModbusDataResponse
  :super ros::object
  :slots (_ret _message _respond_data ))

(defmethod xarm_msgs::GetSetModbusDataResponse
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

(defclass xarm_msgs::GetSetModbusData
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetSetModbusData :md5sum-) "dc342acff6bdfadcdcdb95fac36d565f")
(setf (get xarm_msgs::GetSetModbusData :datatype-) "xarm_msgs/GetSetModbusData")
(setf (get xarm_msgs::GetSetModbusData :request) xarm_msgs::GetSetModbusDataRequest)
(setf (get xarm_msgs::GetSetModbusData :response) xarm_msgs::GetSetModbusDataResponse)

(defmethod xarm_msgs::GetSetModbusDataRequest
  (:response () (instance xarm_msgs::GetSetModbusDataResponse :init)))

(setf (get xarm_msgs::GetSetModbusDataRequest :md5sum-) "dc342acff6bdfadcdcdb95fac36d565f")
(setf (get xarm_msgs::GetSetModbusDataRequest :datatype-) "xarm_msgs/GetSetModbusDataRequest")
(setf (get xarm_msgs::GetSetModbusDataRequest :definition-)
      "# unsigned char data to be sent to tool device through modbus
uint8[] send_data

# Specify the anticipated maximum respond data length in bytes
int16 respond_len

# host id, 9: END RS485, 10: Controller RS485
uint8 host_id

# whether to choose transparent transmission
bool is_transparent_transmission

# whether to use port 503 for communication
# if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission
bool use_503_port

---

int16 ret
string message
uint8[] respond_data
")

(setf (get xarm_msgs::GetSetModbusDataResponse :md5sum-) "dc342acff6bdfadcdcdb95fac36d565f")
(setf (get xarm_msgs::GetSetModbusDataResponse :datatype-) "xarm_msgs/GetSetModbusDataResponse")
(setf (get xarm_msgs::GetSetModbusDataResponse :definition-)
      "# unsigned char data to be sent to tool device through modbus
uint8[] send_data

# Specify the anticipated maximum respond data length in bytes
int16 respond_len

# host id, 9: END RS485, 10: Controller RS485
uint8 host_id

# whether to choose transparent transmission
bool is_transparent_transmission

# whether to use port 503 for communication
# if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission
bool use_503_port

---

int16 ret
string message
uint8[] respond_data
")



(provide :xarm_msgs/GetSetModbusData "dc342acff6bdfadcdcdb95fac36d565f")


