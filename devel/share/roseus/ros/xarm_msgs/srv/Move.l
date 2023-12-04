;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::Move)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'Move (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::MOVE")
  (make-package "XARM_MSGS::MOVE"))
(unless (find-package "XARM_MSGS::MOVEREQUEST")
  (make-package "XARM_MSGS::MOVEREQUEST"))
(unless (find-package "XARM_MSGS::MOVERESPONSE")
  (make-package "XARM_MSGS::MOVERESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::MoveRequest
  :super ros::object
  :slots (_pose _mvvelo _mvacc _mvtime _mvradii ))

(defmethod xarm_msgs::MoveRequest
  (:init
   (&key
    ((:pose __pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mvvelo __mvvelo) 0.0)
    ((:mvacc __mvacc) 0.0)
    ((:mvtime __mvtime) 0.0)
    ((:mvradii __mvradii) 0.0)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _mvvelo (float __mvvelo))
   (setq _mvacc (float __mvacc))
   (setq _mvtime (float __mvtime))
   (setq _mvradii (float __mvradii))
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
  (:mvradii
   (&optional __mvradii)
   (if __mvradii (setq _mvradii __mvradii)) _mvradii)
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
    ;; float32 _mvradii
    4
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
     ;; float32 _mvradii
       (sys::poke _mvradii (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _mvradii
     (setq _mvradii (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::MoveResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::MoveResponse
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

(defclass xarm_msgs::Move
  :super ros::object
  :slots ())

(setf (get xarm_msgs::Move :md5sum-) "149d0c53b84d9002e7f64f44b16335dd")
(setf (get xarm_msgs::Move :datatype-) "xarm_msgs/Move")
(setf (get xarm_msgs::Move :request) xarm_msgs::MoveRequest)
(setf (get xarm_msgs::Move :response) xarm_msgs::MoveResponse)

(defmethod xarm_msgs::MoveRequest
  (:response () (instance xarm_msgs::MoveResponse :init)))

(setf (get xarm_msgs::MoveRequest :md5sum-) "149d0c53b84d9002e7f64f44b16335dd")
(setf (get xarm_msgs::MoveRequest :datatype-) "xarm_msgs/MoveRequest")
(setf (get xarm_msgs::MoveRequest :definition-)
      "# request: command specification for motion executions.
# Units:
#	joint space/angles: radian, radian/s and radian/s^2.
#	Cartesian space: mm, mm/s, and mm/s^2.
#	time: sec

# pose: target coordinate. 
#	For Joint Space target，pose dimention is the number of joints. element as each target joint position.
#	For Cartesian target: pose dimention is 6 for (x, y, z, roll, pitch, yaw)

float32[] pose

# mvvelo: specified maximum velocity during execution. linear or angular velocity 

float32 mvvelo

# mvacc: specified maximum acceleration during execution. linear or angular acceleration.

float32 mvacc

# mvtime: currently do not have any special meaning, please just give it 0. 
# PLEASE NOTE: after firmware version 1.5, For servo_cartesian motion, mvtime will be used as indicator of coordinate system. (0 for BASE coordinate, non-zero for TOOL coordinate)  

float32 mvtime

# mvradii: this is special for move_ineb service, meaning the blending radius between 2 straight path trajectories, 0 for no blend.

float32 mvradii

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")

(setf (get xarm_msgs::MoveResponse :md5sum-) "149d0c53b84d9002e7f64f44b16335dd")
(setf (get xarm_msgs::MoveResponse :datatype-) "xarm_msgs/MoveResponse")
(setf (get xarm_msgs::MoveResponse :definition-)
      "# request: command specification for motion executions.
# Units:
#	joint space/angles: radian, radian/s and radian/s^2.
#	Cartesian space: mm, mm/s, and mm/s^2.
#	time: sec

# pose: target coordinate. 
#	For Joint Space target，pose dimention is the number of joints. element as each target joint position.
#	For Cartesian target: pose dimention is 6 for (x, y, z, roll, pitch, yaw)

float32[] pose

# mvvelo: specified maximum velocity during execution. linear or angular velocity 

float32 mvvelo

# mvacc: specified maximum acceleration during execution. linear or angular acceleration.

float32 mvacc

# mvtime: currently do not have any special meaning, please just give it 0. 
# PLEASE NOTE: after firmware version 1.5, For servo_cartesian motion, mvtime will be used as indicator of coordinate system. (0 for BASE coordinate, non-zero for TOOL coordinate)  

float32 mvtime

# mvradii: this is special for move_ineb service, meaning the blending radius between 2 straight path trajectories, 0 for no blend.

float32 mvradii

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")



(provide :xarm_msgs/Move "149d0c53b84d9002e7f64f44b16335dd")


