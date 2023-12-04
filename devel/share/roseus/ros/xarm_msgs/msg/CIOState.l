;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::CIOState)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'CIOState (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::CIOSTATE")
  (make-package "XARM_MSGS::CIOSTATE"))

(in-package "ROS")
;;//! \htmlinclude CIOState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass xarm_msgs::CIOState
  :super ros::object
  :slots (_header _state _code _input_digitals _output_digitals _input_analogs _output_analogs _input_conf _output_conf ))

(defmethod xarm_msgs::CIOState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:code __code) 0)
    ((:input_digitals __input_digitals) (make-array 2 :initial-element 0 :element-type :integer))
    ((:output_digitals __output_digitals) (make-array 2 :initial-element 0 :element-type :integer))
    ((:input_analogs __input_analogs) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:output_analogs __output_analogs) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:input_conf __input_conf) (make-array 16 :initial-element 0 :element-type :char))
    ((:output_conf __output_conf) (make-array 16 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _code (round __code))
   (setq _input_digitals __input_digitals)
   (setq _output_digitals __output_digitals)
   (setq _input_analogs __input_analogs)
   (setq _output_analogs __output_analogs)
   (setq _input_conf __input_conf)
   (setq _output_conf __output_conf)
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
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:input_digitals
   (&optional __input_digitals)
   (if __input_digitals (setq _input_digitals __input_digitals)) _input_digitals)
  (:output_digitals
   (&optional __output_digitals)
   (if __output_digitals (setq _output_digitals __output_digitals)) _output_digitals)
  (:input_analogs
   (&optional __input_analogs)
   (if __input_analogs (setq _input_analogs __input_analogs)) _input_analogs)
  (:output_analogs
   (&optional __output_analogs)
   (if __output_analogs (setq _output_analogs __output_analogs)) _output_analogs)
  (:input_conf
   (&optional __input_conf)
   (if __input_conf (setq _input_conf __input_conf)) _input_conf)
  (:output_conf
   (&optional __output_conf)
   (if __output_conf (setq _output_conf __output_conf)) _output_conf)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _state
    2
    ;; int16 _code
    2
    ;; uint16[2] _input_digitals
    (* 2    2)
    ;; uint16[2] _output_digitals
    (* 2    2)
    ;; float32[2] _input_analogs
    (* 4    2)
    ;; float32[2] _output_analogs
    (* 4    2)
    ;; uint8[16] _input_conf
    (* 1    16)
    ;; uint8[16] _output_conf
    (* 1    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _state
       (write-word _state s)
     ;; int16 _code
       (write-word _code s)
     ;; uint16[2] _input_digitals
     (dotimes (i 2)
       (write-word (elt _input_digitals i) s)
       )
     ;; uint16[2] _output_digitals
     (dotimes (i 2)
       (write-word (elt _output_digitals i) s)
       )
     ;; float32[2] _input_analogs
     (dotimes (i 2)
       (sys::poke (elt _input_analogs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _output_analogs
     (dotimes (i 2)
       (sys::poke (elt _output_analogs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8[16] _input_conf
     (princ _input_conf s)
     ;; uint8[16] _output_conf
     (princ _output_conf s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _state
     (setq _state (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _code
     (setq _code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16[2] _input_digitals
   (dotimes (i (length _input_digitals))
     (setf (elt _input_digitals i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[2] _output_digitals
   (dotimes (i (length _output_digitals))
     (setf (elt _output_digitals i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; float32[2] _input_analogs
   (dotimes (i (length _input_analogs))
     (setf (elt _input_analogs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _output_analogs
   (dotimes (i (length _output_analogs))
     (setf (elt _output_analogs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8[16] _input_conf
   (setq _input_conf (make-array 16 :element-type :char))
   (replace _input_conf buf :start2 ptr-) (incf ptr- 16)
   ;; uint8[16] _output_conf
   (setq _output_conf (make-array 16 :element-type :char))
   (replace _output_conf buf :start2 ptr-) (incf ptr- 16)
   ;;
   self)
  )

(setf (get xarm_msgs::CIOState :md5sum-) "2a51ddec3163bc4c38eaa278474b2dc1")
(setf (get xarm_msgs::CIOState :datatype-) "xarm_msgs/CIOState")
(setf (get xarm_msgs::CIOState :definition-)
      "Header header

# contorller gpio module state
int16 state

# controller gpio module error code
int16 code

# input_digitals[0]: digital input functional gpio state
# input_digitals[1]: digital input configuring gpio state
#    CI0: (input_digitals[1] >> 0) & 0x0001
#    CI1: (input_digitals[1] >> 1) & 0x0001
#    CI7: (input_digitals[1] >> 7) & 0x0001
#    DI0: (input_digitals[1] >> 8) & 0x0001
#    DI1: (input_digitals[1] >> 9) & 0x0001
#    DI7: (input_digitals[1] >> 15) & 0x0001
uint16[2] input_digitals

# output_digitals[0]: digital output functional gpio state
# output_digitals[1]: digital output configuring gpio state
#    CO0: (output_digitals[1] >> 0) & 0x0001
#    CO1: (output_digitals[1] >> 1) & 0x0001
#    CO7: (output_digitals[1] >> 7) & 0x0001
#    DO0: (output_digitals[1] >> 8) & 0x0001
#    DO1: (output_digitals[1] >> 9) & 0x0001
#    DO7: (output_digitals[1] >> 15) & 0x0001
uint16[2] output_digitals

# input_analogs[0]: the value of AI0
# input_analogs[1]: the value of AI1
float32[2] input_analogs

# output_analogs[0]: the value of AO0
# output_analogs[1]: the value of AO1
float32[2] output_analogs

# digital input functional info
uint8[16] input_conf

# digital output functional info
uint8[16] output_conf
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



(provide :xarm_msgs/CIOState "2a51ddec3163bc4c38eaa278474b2dc1")


