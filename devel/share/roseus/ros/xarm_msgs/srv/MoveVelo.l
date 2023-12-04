;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::MoveVelo)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'MoveVelo (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::MOVEVELO")
  (make-package "XARM_MSGS::MOVEVELO"))
(unless (find-package "XARM_MSGS::MOVEVELOREQUEST")
  (make-package "XARM_MSGS::MOVEVELOREQUEST"))
(unless (find-package "XARM_MSGS::MOVEVELORESPONSE")
  (make-package "XARM_MSGS::MOVEVELORESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::MoveVeloRequest
  :super ros::object
  :slots (_velocities _jnt_sync _coord ))

(defmethod xarm_msgs::MoveVeloRequest
  (:init
   (&key
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:jnt_sync __jnt_sync) 0)
    ((:coord __coord) 0)
    )
   (send-super :init)
   (setq _velocities __velocities)
   (setq _jnt_sync (round __jnt_sync))
   (setq _coord (round __coord))
   self)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:jnt_sync
   (&optional __jnt_sync)
   (if __jnt_sync (setq _jnt_sync __jnt_sync)) _jnt_sync)
  (:coord
   (&optional __coord)
   (if __coord (setq _coord __coord)) _coord)
  (:serialization-length
   ()
   (+
    ;; float32[] _velocities
    (* 4    (length _velocities)) 4
    ;; int16 _jnt_sync
    2
    ;; int16 _coord
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16 _jnt_sync
       (write-word _jnt_sync s)
     ;; int16 _coord
       (write-word _coord s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int16 _jnt_sync
     (setq _jnt_sync (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _coord
     (setq _coord (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::MoveVeloResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::MoveVeloResponse
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

(defclass xarm_msgs::MoveVelo
  :super ros::object
  :slots ())

(setf (get xarm_msgs::MoveVelo :md5sum-) "7ab1cadf314da821f631e86ead48e30d")
(setf (get xarm_msgs::MoveVelo :datatype-) "xarm_msgs/MoveVelo")
(setf (get xarm_msgs::MoveVelo :request) xarm_msgs::MoveVeloRequest)
(setf (get xarm_msgs::MoveVelo :response) xarm_msgs::MoveVeloResponse)

(defmethod xarm_msgs::MoveVeloRequest
  (:response () (instance xarm_msgs::MoveVeloResponse :init)))

(setf (get xarm_msgs::MoveVeloRequest :md5sum-) "7ab1cadf314da821f631e86ead48e30d")
(setf (get xarm_msgs::MoveVeloRequest :datatype-) "xarm_msgs/MoveVeloRequest")
(setf (get xarm_msgs::MoveVeloRequest :definition-)
      "# request: command specification for velocity executions.
# Units:
#	joint space/angles: radian/s
#	Cartesian space: mm/s, radian/s.

# velocities: the velocity list of the joints/tcp
#   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]
#   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]
float32[] velocities

# jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.
int16 jnt_sync

# coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)
int16 coord

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")

(setf (get xarm_msgs::MoveVeloResponse :md5sum-) "7ab1cadf314da821f631e86ead48e30d")
(setf (get xarm_msgs::MoveVeloResponse :datatype-) "xarm_msgs/MoveVeloResponse")
(setf (get xarm_msgs::MoveVeloResponse :definition-)
      "# request: command specification for velocity executions.
# Units:
#	joint space/angles: radian/s
#	Cartesian space: mm/s, radian/s.

# velocities: the velocity list of the joints/tcp
#   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]
#   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]
float32[] velocities

# jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.
int16 jnt_sync

# coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)
int16 coord

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret

string message
")



(provide :xarm_msgs/MoveVelo "7ab1cadf314da821f631e86ead48e30d")


