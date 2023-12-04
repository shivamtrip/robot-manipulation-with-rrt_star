;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetString)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetString (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETSTRING")
  (make-package "XARM_MSGS::SETSTRING"))
(unless (find-package "XARM_MSGS::SETSTRINGREQUEST")
  (make-package "XARM_MSGS::SETSTRINGREQUEST"))
(unless (find-package "XARM_MSGS::SETSTRINGRESPONSE")
  (make-package "XARM_MSGS::SETSTRINGRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetStringRequest
  :super ros::object
  :slots (_str_data _timeout ))

(defmethod xarm_msgs::SetStringRequest
  (:init
   (&key
    ((:str_data __str_data) "")
    ((:timeout __timeout) 0.0)
    )
   (send-super :init)
   (setq _str_data (string __str_data))
   (setq _timeout (float __timeout))
   self)
  (:str_data
   (&optional __str_data)
   (if __str_data (setq _str_data __str_data)) _str_data)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:serialization-length
   ()
   (+
    ;; string _str_data
    4 (length _str_data)
    ;; float32 _timeout
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _str_data
       (write-long (length _str_data) s) (princ _str_data s)
     ;; float32 _timeout
       (sys::poke _timeout (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _str_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _str_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _timeout
     (setq _timeout (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xarm_msgs::SetStringResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetStringResponse
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

(defclass xarm_msgs::SetString
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetString :md5sum-) "8ef44a329258a19c2b77a7a598761c55")
(setf (get xarm_msgs::SetString :datatype-) "xarm_msgs/SetString")
(setf (get xarm_msgs::SetString :request) xarm_msgs::SetStringRequest)
(setf (get xarm_msgs::SetString :response) xarm_msgs::SetStringResponse)

(defmethod xarm_msgs::SetStringRequest
  (:response () (instance xarm_msgs::SetStringResponse :init)))

(setf (get xarm_msgs::SetStringRequest :md5sum-) "8ef44a329258a19c2b77a7a598761c55")
(setf (get xarm_msgs::SetStringRequest :datatype-) "xarm_msgs/SetStringRequest")
(setf (get xarm_msgs::SetStringRequest :definition-)
      "# request: set string command parameters.
# str_data: string parameter to be set.

string str_data
float32 timeout

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")

(setf (get xarm_msgs::SetStringResponse :md5sum-) "8ef44a329258a19c2b77a7a598761c55")
(setf (get xarm_msgs::SetStringResponse :datatype-) "xarm_msgs/SetStringResponse")
(setf (get xarm_msgs::SetStringResponse :definition-)
      "# request: set string command parameters.
# str_data: string parameter to be set.

string str_data
float32 timeout

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")



(provide :xarm_msgs/SetString "8ef44a329258a19c2b77a7a598761c55")


