;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::FtIdenLoad)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'FtIdenLoad (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::FTIDENLOAD")
  (make-package "XARM_MSGS::FTIDENLOAD"))
(unless (find-package "XARM_MSGS::FTIDENLOADREQUEST")
  (make-package "XARM_MSGS::FTIDENLOADREQUEST"))
(unless (find-package "XARM_MSGS::FTIDENLOADRESPONSE")
  (make-package "XARM_MSGS::FTIDENLOADRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::FtIdenLoadRequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::FtIdenLoadRequest
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

(defclass xarm_msgs::FtIdenLoadResponse
  :super ros::object
  :slots (_ret _message _datas ))

(defmethod xarm_msgs::FtIdenLoadResponse
  (:init
   (&key
    ((:ret __ret) 0)
    ((:message __message) "")
    ((:datas __datas) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _ret (round __ret))
   (setq _message (string __message))
   (setq _datas __datas)
   self)
  (:ret
   (&optional __ret)
   (if __ret (setq _ret __ret)) _ret)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:datas
   (&optional __datas)
   (if __datas (setq _datas __datas)) _datas)
  (:serialization-length
   ()
   (+
    ;; int16 _ret
    2
    ;; string _message
    4 (length _message)
    ;; float32[] _datas
    (* 4    (length _datas)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _ret
       (write-word _ret s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32[] _datas
     (write-long (length _datas) s)
     (dotimes (i (length _datas))
       (sys::poke (elt _datas i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _ret
     (setq _ret (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32[] _datas
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _datas (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _datas i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass xarm_msgs::FtIdenLoad
  :super ros::object
  :slots ())

(setf (get xarm_msgs::FtIdenLoad :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::FtIdenLoad :datatype-) "xarm_msgs/FtIdenLoad")
(setf (get xarm_msgs::FtIdenLoad :request) xarm_msgs::FtIdenLoadRequest)
(setf (get xarm_msgs::FtIdenLoad :response) xarm_msgs::FtIdenLoadResponse)

(defmethod xarm_msgs::FtIdenLoadRequest
  (:response () (instance xarm_msgs::FtIdenLoadResponse :init)))

(setf (get xarm_msgs::FtIdenLoadRequest :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::FtIdenLoadRequest :datatype-) "xarm_msgs/FtIdenLoadRequest")
(setf (get xarm_msgs::FtIdenLoadRequest :definition-)
      "# This format is suitable for the following services
#   - ft_sensor_iden_load

---

int16 ret
string message

# the result of identification
#   ft_sensor_iden_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]
float32[] datas
")

(setf (get xarm_msgs::FtIdenLoadResponse :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::FtIdenLoadResponse :datatype-) "xarm_msgs/FtIdenLoadResponse")
(setf (get xarm_msgs::FtIdenLoadResponse :definition-)
      "# This format is suitable for the following services
#   - ft_sensor_iden_load

---

int16 ret
string message

# the result of identification
#   ft_sensor_iden_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]
float32[] datas
")



(provide :xarm_msgs/FtIdenLoad "9470c18b0944f559a19d029d524ee6b1")


