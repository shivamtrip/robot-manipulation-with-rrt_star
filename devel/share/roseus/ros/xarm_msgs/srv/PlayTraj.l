;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::PlayTraj)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'PlayTraj (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::PLAYTRAJ")
  (make-package "XARM_MSGS::PLAYTRAJ"))
(unless (find-package "XARM_MSGS::PLAYTRAJREQUEST")
  (make-package "XARM_MSGS::PLAYTRAJREQUEST"))
(unless (find-package "XARM_MSGS::PLAYTRAJRESPONSE")
  (make-package "XARM_MSGS::PLAYTRAJRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::PlayTrajRequest
  :super ros::object
  :slots (_traj_file _repeat_times _speed_factor ))

(defmethod xarm_msgs::PlayTrajRequest
  (:init
   (&key
    ((:traj_file __traj_file) "")
    ((:repeat_times __repeat_times) 0)
    ((:speed_factor __speed_factor) 0)
    )
   (send-super :init)
   (setq _traj_file (string __traj_file))
   (setq _repeat_times (round __repeat_times))
   (setq _speed_factor (round __speed_factor))
   self)
  (:traj_file
   (&optional __traj_file)
   (if __traj_file (setq _traj_file __traj_file)) _traj_file)
  (:repeat_times
   (&optional __repeat_times)
   (if __repeat_times (setq _repeat_times __repeat_times)) _repeat_times)
  (:speed_factor
   (&optional __speed_factor)
   (if __speed_factor (setq _speed_factor __speed_factor)) _speed_factor)
  (:serialization-length
   ()
   (+
    ;; string _traj_file
    4 (length _traj_file)
    ;; int16 _repeat_times
    2
    ;; int16 _speed_factor
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _traj_file
       (write-long (length _traj_file) s) (princ _traj_file s)
     ;; int16 _repeat_times
       (write-word _repeat_times s)
     ;; int16 _speed_factor
       (write-word _speed_factor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _traj_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _traj_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _repeat_times
     (setq _repeat_times (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _speed_factor
     (setq _speed_factor (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xarm_msgs::PlayTrajResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::PlayTrajResponse
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

(defclass xarm_msgs::PlayTraj
  :super ros::object
  :slots ())

(setf (get xarm_msgs::PlayTraj :md5sum-) "80c963774ceddb6d2f6133dfc972a9dd")
(setf (get xarm_msgs::PlayTraj :datatype-) "xarm_msgs/PlayTraj")
(setf (get xarm_msgs::PlayTraj :request) xarm_msgs::PlayTrajRequest)
(setf (get xarm_msgs::PlayTraj :response) xarm_msgs::PlayTrajResponse)

(defmethod xarm_msgs::PlayTrajRequest
  (:response () (instance xarm_msgs::PlayTrajResponse :init)))

(setf (get xarm_msgs::PlayTrajRequest :md5sum-) "80c963774ceddb6d2f6133dfc972a9dd")
(setf (get xarm_msgs::PlayTrajRequest :datatype-) "xarm_msgs/PlayTrajRequest")
(setf (get xarm_msgs::PlayTrajRequest :definition-)
      "# request: set multiple integer parameters.
# traj_file: file name of trajectory record, do not forget \".traj\" suffix
# repeat_times: how many times will be played
# speed_factor: supported values are 1 (original speed), 2 (2x speed), 4 (4x speed, not recommended). 

string traj_file
int16 repeat_times
int16 speed_factor

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")

(setf (get xarm_msgs::PlayTrajResponse :md5sum-) "80c963774ceddb6d2f6133dfc972a9dd")
(setf (get xarm_msgs::PlayTrajResponse :datatype-) "xarm_msgs/PlayTrajResponse")
(setf (get xarm_msgs::PlayTrajResponse :definition-)
      "# request: set multiple integer parameters.
# traj_file: file name of trajectory record, do not forget \".traj\" suffix
# repeat_times: how many times will be played
# speed_factor: supported values are 1 (original speed), 2 (2x speed), 4 (4x speed, not recommended). 

string traj_file
int16 repeat_times
int16 speed_factor

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")



(provide :xarm_msgs/PlayTraj "80c963774ceddb6d2f6133dfc972a9dd")


