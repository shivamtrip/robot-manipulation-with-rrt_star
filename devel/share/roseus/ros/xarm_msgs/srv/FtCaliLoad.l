;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::FtCaliLoad)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'FtCaliLoad (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::FTCALILOAD")
  (make-package "XARM_MSGS::FTCALILOAD"))
(unless (find-package "XARM_MSGS::FTCALILOADREQUEST")
  (make-package "XARM_MSGS::FTCALILOADREQUEST"))
(unless (find-package "XARM_MSGS::FTCALILOADRESPONSE")
  (make-package "XARM_MSGS::FTCALILOADRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::FtCaliLoadRequest
  :super ros::object
  :slots (_datas _association_setting_tcp_load ))

(defmethod xarm_msgs::FtCaliLoadRequest
  (:init
   (&key
    ((:datas __datas) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:association_setting_tcp_load __association_setting_tcp_load) nil)
    )
   (send-super :init)
   (setq _datas __datas)
   (setq _association_setting_tcp_load __association_setting_tcp_load)
   self)
  (:datas
   (&optional __datas)
   (if __datas (setq _datas __datas)) _datas)
  (:association_setting_tcp_load
   (&optional (__association_setting_tcp_load :null))
   (if (not (eq __association_setting_tcp_load :null)) (setq _association_setting_tcp_load __association_setting_tcp_load)) _association_setting_tcp_load)
  (:serialization-length
   ()
   (+
    ;; float32[] _datas
    (* 4    (length _datas)) 4
    ;; bool _association_setting_tcp_load
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _datas
     (write-long (length _datas) s)
     (dotimes (i (length _datas))
       (sys::poke (elt _datas i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _association_setting_tcp_load
       (if _association_setting_tcp_load (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _datas
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _datas (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _datas i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _association_setting_tcp_load
     (setq _association_setting_tcp_load (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xarm_msgs::FtCaliLoadResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::FtCaliLoadResponse
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

(defclass xarm_msgs::FtCaliLoad
  :super ros::object
  :slots ())

(setf (get xarm_msgs::FtCaliLoad :md5sum-) "002aec91323cfbf6a70a5f91f59ae0cf")
(setf (get xarm_msgs::FtCaliLoad :datatype-) "xarm_msgs/FtCaliLoad")
(setf (get xarm_msgs::FtCaliLoad :request) xarm_msgs::FtCaliLoadRequest)
(setf (get xarm_msgs::FtCaliLoad :response) xarm_msgs::FtCaliLoadResponse)

(defmethod xarm_msgs::FtCaliLoadRequest
  (:response () (instance xarm_msgs::FtCaliLoadResponse :init)))

(setf (get xarm_msgs::FtCaliLoadRequest :md5sum-) "002aec91323cfbf6a70a5f91f59ae0cf")
(setf (get xarm_msgs::FtCaliLoadRequest :datatype-) "xarm_msgs/FtCaliLoadRequest")
(setf (get xarm_msgs::FtCaliLoadRequest :definition-)
      "# This format is suitable for the following services
#   - ft_sensor_cali_load

# iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
float32[] datas

# whether to convert the paramster to the corresponding tcp load and set
bool association_setting_tcp_load
# float32 m   0.325
# float32 x   -17.0
# float32 y   9.0
# float32 z   11.8

---

int16 ret
string message

")

(setf (get xarm_msgs::FtCaliLoadResponse :md5sum-) "002aec91323cfbf6a70a5f91f59ae0cf")
(setf (get xarm_msgs::FtCaliLoadResponse :datatype-) "xarm_msgs/FtCaliLoadResponse")
(setf (get xarm_msgs::FtCaliLoadResponse :definition-)
      "# This format is suitable for the following services
#   - ft_sensor_cali_load

# iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
float32[] datas

# whether to convert the paramster to the corresponding tcp load and set
bool association_setting_tcp_load
# float32 m   0.325
# float32 x   -17.0
# float32 y   9.0
# float32 z   11.8

---

int16 ret
string message

")



(provide :xarm_msgs/FtCaliLoad "002aec91323cfbf6a70a5f91f59ae0cf")


