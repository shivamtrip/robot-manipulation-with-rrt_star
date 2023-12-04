;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetLoad)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetLoad (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETLOAD")
  (make-package "XARM_MSGS::SETLOAD"))
(unless (find-package "XARM_MSGS::SETLOADREQUEST")
  (make-package "XARM_MSGS::SETLOADREQUEST"))
(unless (find-package "XARM_MSGS::SETLOADRESPONSE")
  (make-package "XARM_MSGS::SETLOADRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetLoadRequest
  :super ros::object
  :slots (_mass _xc _yc _zc ))

(defmethod xarm_msgs::SetLoadRequest
  (:init
   (&key
    ((:mass __mass) 0.0)
    ((:xc __xc) 0.0)
    ((:yc __yc) 0.0)
    ((:zc __zc) 0.0)
    )
   (send-super :init)
   (setq _mass (float __mass))
   (setq _xc (float __xc))
   (setq _yc (float __yc))
   (setq _zc (float __zc))
   self)
  (:mass
   (&optional __mass)
   (if __mass (setq _mass __mass)) _mass)
  (:xc
   (&optional __xc)
   (if __xc (setq _xc __xc)) _xc)
  (:yc
   (&optional __yc)
   (if __yc (setq _yc __yc)) _yc)
  (:zc
   (&optional __zc)
   (if __zc (setq _zc __zc)) _zc)
  (:serialization-length
   ()
   (+
    ;; float32 _mass
    4
    ;; float32 _xc
    4
    ;; float32 _yc
    4
    ;; float32 _zc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _mass
       (sys::poke _mass (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xc
       (sys::poke _xc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yc
       (sys::poke _yc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zc
       (sys::poke _zc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _mass
     (setq _mass (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xc
     (setq _xc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yc
     (setq _yc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zc
     (setq _zc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::SetLoadResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetLoadResponse
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

(defclass xarm_msgs::SetLoad
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetLoad :md5sum-) "a61cba8401ac162ae54cbe464f360d3c")
(setf (get xarm_msgs::SetLoad :datatype-) "xarm_msgs/SetLoad")
(setf (get xarm_msgs::SetLoad :request) xarm_msgs::SetLoadRequest)
(setf (get xarm_msgs::SetLoad :response) xarm_msgs::SetLoadResponse)

(defmethod xarm_msgs::SetLoadRequest
  (:response () (instance xarm_msgs::SetLoadResponse :init)))

(setf (get xarm_msgs::SetLoadRequest :md5sum-) "a61cba8401ac162ae54cbe464f360d3c")
(setf (get xarm_msgs::SetLoadRequest :datatype-) "xarm_msgs/SetLoadRequest")
(setf (get xarm_msgs::SetLoadRequest :definition-)
      "# to set robot load parameters, based on initial Tool Frame located at flange center.
# Load mass (kg)
float32 mass

# Load Center of Mass (mm), with respect to intial Tool Frame
float32 xc
float32 yc
float32 zc

---

int16 ret
string message

")

(setf (get xarm_msgs::SetLoadResponse :md5sum-) "a61cba8401ac162ae54cbe464f360d3c")
(setf (get xarm_msgs::SetLoadResponse :datatype-) "xarm_msgs/SetLoadResponse")
(setf (get xarm_msgs::SetLoadResponse :definition-)
      "# to set robot load parameters, based on initial Tool Frame located at flange center.
# Load mass (kg)
float32 mass

# Load Center of Mass (mm), with respect to intial Tool Frame
float32 xc
float32 yc
float32 zc

---

int16 ret
string message

")



(provide :xarm_msgs/SetLoad "a61cba8401ac162ae54cbe464f360d3c")


