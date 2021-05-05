;; Auto-generated. Do not edit!


(when (boundp 'landmarks_msg::Landmark_Msg)
  (if (not (find-package "LANDMARKS_MSG"))
    (make-package "LANDMARKS_MSG"))
  (shadow 'Landmark_Msg (find-package "LANDMARKS_MSG")))
(unless (find-package "LANDMARKS_MSG::LANDMARK_MSG")
  (make-package "LANDMARKS_MSG::LANDMARK_MSG"))

(in-package "ROS")
;;//! \htmlinclude Landmark_Msg.msg.html


(defclass landmarks_msg::Landmark_Msg
  :super ros::object
  :slots (_label _x _y _s_x _s_y ))

(defmethod landmarks_msg::Landmark_Msg
  (:init
   (&key
    ((:label __label) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:s_x __s_x) 0.0)
    ((:s_y __s_y) 0.0)
    )
   (send-super :init)
   (setq _label (round __label))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _s_x (float __s_x))
   (setq _s_y (float __s_y))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:s_x
   (&optional __s_x)
   (if __s_x (setq _s_x __s_x)) _s_x)
  (:s_y
   (&optional __s_y)
   (if __s_y (setq _s_y __s_y)) _s_y)
  (:serialization-length
   ()
   (+
    ;; uint64 _label
    8
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _s_x
    4
    ;; float32 _s_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _label
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _label (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _label) (= (length (_label . bv)) 2)) ;; bignum
              (write-long (ash (elt (_label . bv) 0) 0) s)
              (write-long (ash (elt (_label . bv) 1) -1) s))
             ((and (class _label) (= (length (_label . bv)) 1)) ;; big1
              (write-long (elt (_label . bv) 0) s)
              (write-long (if (>= _label 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _label s)(write-long (if (>= _label 0) 0 #xffffffff) s)))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s_x
       (sys::poke _s_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _s_y
       (sys::poke _s_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _label
     
#+(or :alpha :irix6 :x86_64)
      (setf _label (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _label (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s_x
     (setq _s_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _s_y
     (setq _s_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get landmarks_msg::Landmark_Msg :md5sum-) "37586ccc5f4ed3186510933e2fe487e9")
(setf (get landmarks_msg::Landmark_Msg :datatype-) "landmarks_msg/Landmark_Msg")
(setf (get landmarks_msg::Landmark_Msg :definition-)
      "uint64 label
float32 x
float32 y
float32 s_x 
float32 s_y 





")



(provide :landmarks_msg/Landmark_Msg "37586ccc5f4ed3186510933e2fe487e9")


