;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::RobotMsg)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'RobotMsg (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::ROBOTMSG")
  (make-package "XARM_MSGS::ROBOTMSG"))

(in-package "ROS")
;;//! \htmlinclude RobotMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass xarm_msgs::RobotMsg
  :super ros::object
  :slots (_header _state _mode _cmdnum _mt_brake _mt_able _err _warn _angle _pose _offset ))

(defmethod xarm_msgs::RobotMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:mode __mode) 0)
    ((:cmdnum __cmdnum) 0)
    ((:mt_brake __mt_brake) 0)
    ((:mt_able __mt_able) 0)
    ((:err __err) 0)
    ((:warn __warn) 0)
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose __pose) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:offset __offset) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _mode (round __mode))
   (setq _cmdnum (round __cmdnum))
   (setq _mt_brake (round __mt_brake))
   (setq _mt_able (round __mt_able))
   (setq _err (round __err))
   (setq _warn (round __warn))
   (setq _angle __angle)
   (setq _pose __pose)
   (setq _offset __offset)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:cmdnum
   (&optional __cmdnum)
   (if __cmdnum (setq _cmdnum __cmdnum)) _cmdnum)
  (:mt_brake
   (&optional __mt_brake)
   (if __mt_brake (setq _mt_brake __mt_brake)) _mt_brake)
  (:mt_able
   (&optional __mt_able)
   (if __mt_able (setq _mt_able __mt_able)) _mt_able)
  (:err
   (&optional __err)
   (if __err (setq _err __err)) _err)
  (:warn
   (&optional __warn)
   (if __warn (setq _warn __warn)) _warn)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _state
    2
    ;; int16 _mode
    2
    ;; int16 _cmdnum
    2
    ;; int16 _mt_brake
    2
    ;; int16 _mt_able
    2
    ;; int16 _err
    2
    ;; int16 _warn
    2
    ;; float32[] _angle
    (* 4    (length _angle)) 4
    ;; float32[6] _pose
    (* 4    6)
    ;; float32[6] _offset
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _state
       (write-word _state s)
     ;; int16 _mode
       (write-word _mode s)
     ;; int16 _cmdnum
       (write-word _cmdnum s)
     ;; int16 _mt_brake
       (write-word _mt_brake s)
     ;; int16 _mt_able
       (write-word _mt_able s)
     ;; int16 _err
       (write-word _err s)
     ;; int16 _warn
       (write-word _warn s)
     ;; float32[] _angle
     (write-long (length _angle) s)
     (dotimes (i (length _angle))
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _pose
     (dotimes (i 6)
       (sys::poke (elt _pose i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[6] _offset
     (dotimes (i 6)
       (sys::poke (elt _offset i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _state
     (setq _state (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mode
     (setq _mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _cmdnum
     (setq _cmdnum (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mt_brake
     (setq _mt_brake (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mt_able
     (setq _mt_able (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _err
     (setq _err (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _warn
     (setq _warn (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[] _angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[6] _pose
   (dotimes (i (length _pose))
     (setf (elt _pose i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[6] _offset
   (dotimes (i (length _offset))
     (setf (elt _offset i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get xarm_msgs::RobotMsg :md5sum-) "b796af25d2a8912964fb162d2141ac39")
(setf (get xarm_msgs::RobotMsg :datatype-) "xarm_msgs/RobotMsg")
(setf (get xarm_msgs::RobotMsg :definition-)
      "# feedback information of the controlled robot
Header header

# state of robot:
#	1: RUNNING, executing motion command.
#	2: SLEEPING, not in execution, but ready to move.
#	3: PAUSED, paused in the middle of unfinished motion.
#	4: STOPPED, not ready for any motion commands.
#	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.
int16 state

# mode of robot:
#	0 for POSITION mode.(position control by xarm controller box, execute api standard commands)
#	1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)
#	2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)
int16 mode

# cmdnum: number of commands waiting in the buffer.
int16 cmdnum

# mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled
int16 mt_brake

# mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled
int16 mt_able

# error code (if non-zero)
int16 err

# warning code (if non-zero)
int16 warn

# current joint angles expressed in radian.
float32[] angle

# current TCP Cartesian position expressed in mm(position), radian(orientation)  
float32[6] pose

# TCP offset from center of flange, with respect to tool frame.
float32[6] offset
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :xarm_msgs/RobotMsg "b796af25d2a8912964fb162d2141ac39")


