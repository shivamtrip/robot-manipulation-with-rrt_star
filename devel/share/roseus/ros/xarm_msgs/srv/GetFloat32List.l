;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::GetFloat32List)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'GetFloat32List (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::GETFLOAT32LIST")
  (make-package "XARM_MSGS::GETFLOAT32LIST"))
(unless (find-package "XARM_MSGS::GETFLOAT32LISTREQUEST")
  (make-package "XARM_MSGS::GETFLOAT32LISTREQUEST"))
(unless (find-package "XARM_MSGS::GETFLOAT32LISTRESPONSE")
  (make-package "XARM_MSGS::GETFLOAT32LISTRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::GetFloat32ListRequest
  :super ros::object
  :slots ())

(defmethod xarm_msgs::GetFloat32ListRequest
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

(defclass xarm_msgs::GetFloat32ListResponse
  :super ros::object
  :slots (_ret _message _datas ))

(defmethod xarm_msgs::GetFloat32ListResponse
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

(defclass xarm_msgs::GetFloat32List
  :super ros::object
  :slots ())

(setf (get xarm_msgs::GetFloat32List :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::GetFloat32List :datatype-) "xarm_msgs/GetFloat32List")
(setf (get xarm_msgs::GetFloat32List :request) xarm_msgs::GetFloat32ListRequest)
(setf (get xarm_msgs::GetFloat32List :response) xarm_msgs::GetFloat32ListResponse)

(defmethod xarm_msgs::GetFloat32ListRequest
  (:response () (instance xarm_msgs::GetFloat32ListResponse :init)))

(setf (get xarm_msgs::GetFloat32ListRequest :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::GetFloat32ListRequest :datatype-) "xarm_msgs/GetFloat32ListRequest")
(setf (get xarm_msgs::GetFloat32ListRequest :definition-)
      "# This format is suitable for the following services
#   - get_position
#   - get_servo_angle
#   - get_position_aa

---

int16 ret
string message

float32[] datas
")

(setf (get xarm_msgs::GetFloat32ListResponse :md5sum-) "9470c18b0944f559a19d029d524ee6b1")
(setf (get xarm_msgs::GetFloat32ListResponse :datatype-) "xarm_msgs/GetFloat32ListResponse")
(setf (get xarm_msgs::GetFloat32ListResponse :definition-)
      "# This format is suitable for the following services
#   - get_position
#   - get_servo_angle
#   - get_position_aa

---

int16 ret
string message

float32[] datas
")



(provide :xarm_msgs/GetFloat32List "9470c18b0944f559a19d029d524ee6b1")


