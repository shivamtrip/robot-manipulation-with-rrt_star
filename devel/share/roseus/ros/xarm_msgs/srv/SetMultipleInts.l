;; Auto-generated. Do not edit!


(when (boundp 'xarm_msgs::SetMultipleInts)
  (if (not (find-package "XARM_MSGS"))
    (make-package "XARM_MSGS"))
  (shadow 'SetMultipleInts (find-package "XARM_MSGS")))
(unless (find-package "XARM_MSGS::SETMULTIPLEINTS")
  (make-package "XARM_MSGS::SETMULTIPLEINTS"))
(unless (find-package "XARM_MSGS::SETMULTIPLEINTSREQUEST")
  (make-package "XARM_MSGS::SETMULTIPLEINTSREQUEST"))
(unless (find-package "XARM_MSGS::SETMULTIPLEINTSRESPONSE")
  (make-package "XARM_MSGS::SETMULTIPLEINTSRESPONSE"))

(in-package "ROS")





(defclass xarm_msgs::SetMultipleIntsRequest
  :super ros::object
  :slots (_data_int ))

(defmethod xarm_msgs::SetMultipleIntsRequest
  (:init
   (&key
    ((:data_int __data_int) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data_int __data_int)
   self)
  (:data_int
   (&optional __data_int)
   (if __data_int (setq _data_int __data_int)) _data_int)
  (:serialization-length
   ()
   (+
    ;; int16[] _data_int
    (* 2    (length _data_int)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _data_int
     (write-long (length _data_int) s)
     (dotimes (i (length _data_int))
       (write-word (elt _data_int i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _data_int
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data_int (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data_int i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(defclass xarm_msgs::SetMultipleIntsResponse
  :super ros::object
  :slots (_ret _message ))

(defmethod xarm_msgs::SetMultipleIntsResponse
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

(defclass xarm_msgs::SetMultipleInts
  :super ros::object
  :slots ())

(setf (get xarm_msgs::SetMultipleInts :md5sum-) "72a7c6b1ca8071c36cdc4733c55fc7f3")
(setf (get xarm_msgs::SetMultipleInts :datatype-) "xarm_msgs/SetMultipleInts")
(setf (get xarm_msgs::SetMultipleInts :request) xarm_msgs::SetMultipleIntsRequest)
(setf (get xarm_msgs::SetMultipleInts :response) xarm_msgs::SetMultipleIntsResponse)

(defmethod xarm_msgs::SetMultipleIntsRequest
  (:response () (instance xarm_msgs::SetMultipleIntsResponse :init)))

(setf (get xarm_msgs::SetMultipleIntsRequest :md5sum-) "72a7c6b1ca8071c36cdc4733c55fc7f3")
(setf (get xarm_msgs::SetMultipleIntsRequest :datatype-) "xarm_msgs/SetMultipleIntsRequest")
(setf (get xarm_msgs::SetMultipleIntsRequest :definition-)
      "# request: set multiple integer parameters.
# data_int: int values to be set.

int16[] data_int

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")

(setf (get xarm_msgs::SetMultipleIntsResponse :md5sum-) "72a7c6b1ca8071c36cdc4733c55fc7f3")
(setf (get xarm_msgs::SetMultipleIntsResponse :datatype-) "xarm_msgs/SetMultipleIntsResponse")
(setf (get xarm_msgs::SetMultipleIntsResponse :definition-)
      "# request: set multiple integer parameters.
# data_int: int values to be set.

int16[] data_int

---

# response: 
#	ret is 0 for successful execution and others for errors or warnings occured
#	message is a string returned by function, indicating execution status.

int16 ret
string message
")



(provide :xarm_msgs/SetMultipleInts "72a7c6b1ca8071c36cdc4733c55fc7f3")


