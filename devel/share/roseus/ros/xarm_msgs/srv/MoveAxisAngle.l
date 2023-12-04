;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::MoveAxisAngle)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'MoveAxisAngle (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::MOVEAXISANGLE")
  (make-package "XARM_MSGS::MOVEAXISANGLE"))
(unless (find-package "XARM_MSGS::MOVEAXISANGLEREQUEST")
  (make-package "XARM_MSGS::MOVEAXISANGLEREQUEST"))
(unless (find-package "XARM_MSGS::MOVEAXISANGLERESPONSE")
  (make-package "XARM_MSGS::MOVEAXISANGLERESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::MoveAxisAngleRequest
  :super ros::object
  :slots (_pose _mvvelo _mvacc _mvtime _coord _relative ))

(defmethod xarm_msgs::MoveAxisAngleRequest
  (:init
   (&key
    ((:pose __pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mvvelo __mvvelo) 0.0)
    ((:mvacc __mvacc) 0.0)
    ((:mvtime __mvtime) 0.0)
    ((:coord __coord) 0)
    ((:relative __relative) 0)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _mvvelo (float __mvvelo))
   (setq _mvacc (float __mvacc))
   (setq _mvtime (float __mvtime))
   (setq _coord (round __coord))
   (setq _relative (round __relative))
   self)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:mvvelo
   (&optional __mvvelo)
   (if __mvvelo (setq _mvvelo __mvvelo)) _mvvelo)
  (:mvacc
   (&optional __mvacc)
   (if __mvacc (setq _mvacc __mvacc)) _mvacc)
  (:mvtime
   (&optional __mvtime)
   (if __mvtime (setq _mvtime __mvtime)) _mvtime)
  (:coord
   (&optional __coord)
   (if __coord (setq _coord __coord)) _coord)
  (:relative
   (&optional __relative)
   (if __relative (setq _relative __relative)) _relative)
  (:serialization-length
   ()
   (+
    ;; float32[] _pose
    (* 4    (length _pose)) 4
    ;; float32 _mvvelo
    4
    ;; float32 _mvacc
    4
    ;; float32 _mvtime
    4
    ;; int16 _coord
    2
    ;; int16 _relative
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _pose
     (write-long (length _pose) s)
     (dotimes (i (length _pose))
       (sys::poke (elt _pose i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _mvvelo
       (sys::poke _mvvelo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mvacc
       (sys::poke _mvacc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mvtime
       (sys::poke _mvtime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _coord
       (write-word _coord s)
     ;; int16 _relative
       (write-word _relative s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pose i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _mvvelo
     (setq _mvvelo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mvacc
     (setq _mvacc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mvtime
     (setq _mvtime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _coord
     (setq _coord (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _relative
     (setq _relative (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::MoveAxisAngleResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::MoveAxisAngleResponse
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

(defclass xarm_msgs::MoveAxisAngle
  :super ros::object
  :slots ())

(setf (get xarm_msgs::MoveAxisAngle :md5sum-) "e3ce015929065df69fd9351b8d2c8407")
(setf (get xarm_msgs::MoveAxisAngle :datatype-) "xarm_msgs/MoveAxisAngle")
(setf (get xarm_msgs::MoveAxisAngle :request) xarm_msgs::MoveAxisAngleRequest)
(setf (get xarm_msgs::MoveAxisAngle :response) xarm_msgs::MoveAxisAngleResponse)

(defmethod xarm_msgs::MoveAxisAngleRequest
  (:response () (instance xarm_msgs::MoveAxisAngleResponse :init)))

(setf (get xarm_msgs::MoveAxisAngleRequest :md5sum-) "e3ce015929065df69fd9351b8d2c8407")
(setf (get xarm_msgs::MoveAxisAngleRequest :datatype-) "xarm_msgs/MoveAxisAngleRequest")
(setf (get xarm_msgs::MoveAxisAngleRequest :definition-)
      "# request: command specification for axis-angle motion executions.
# Units:
#	Linear: mm, mm/s, and mm/s^2.
#   Angular: radian, radian/s and radian/s^2.
#	time: sec

# pose: target coordinate. 
#	For Cartesian target: pose dimention is 6 for (x, y, z, rx, ry, rz)

float32[] pose

# mvvelo: specified maximum velocity during execution. linear or angular velocity 

float32 mvvelo

# mvacc: specified maximum acceleration during execution. linear or angular acceleration.

float32 mvacc

# mvtime: currently do not have any special meaning, please just give it 0. 

float32 mvtime

# coord: motion coordinate system indicator, base (0) or tool(1) coordinate.

int16 coord

# relative: indicator of given target is relative (1) or not (0, absolute)

int16 relative

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")

(setf (get xarm_msgs::MoveAxisAngleResponse :md5sum-) "e3ce015929065df69fd9351b8d2c8407")
(setf (get xarm_msgs::MoveAxisAngleResponse :datatype-) "xarm_msgs/MoveAxisAngleResponse")
(setf (get xarm_msgs::MoveAxisAngleResponse :definition-)
      "# request: command specification for axis-angle motion executions.
# Units:
#	Linear: mm, mm/s, and mm/s^2.
#   Angular: radian, radian/s and radian/s^2.
#	time: sec

# pose: target coordinate. 
#	For Cartesian target: pose dimention is 6 for (x, y, z, rx, ry, rz)

float32[] pose

# mvvelo: specified maximum velocity during execution. linear or angular velocity 

float32 mvvelo

# mvacc: specified maximum acceleration during execution. linear or angular acceleration.

float32 mvacc

# mvtime: currently do not have any special meaning, please just give it 0. 

float32 mvtime

# coord: motion coordinate system indicator, base (0) or tool(1) coordinate.

int16 coord

# relative: indicator of given target is relative (1) or not (0, absolute)

int16 relative

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")



(provide :xarm_msgs/MoveAxisAngle "e3ce015929065df69fd9351b8d2c8407")


