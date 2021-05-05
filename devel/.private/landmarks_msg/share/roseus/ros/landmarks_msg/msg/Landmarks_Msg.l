;; Auto-generated. Do not edit!


(when (boundp 'landmarks_msg::Landmarks_Msg)
  (if (not (find-package "LANDMARKS_MSG"))
    (make-package "LANDMARKS_MSG"))
  (shadow 'Landmarks_Msg (find-package "LANDMARKS_MSG")))
(unless (find-package "LANDMARKS_MSG::LANDMARKS_MSG")
  (make-package "LANDMARKS_MSG::LANDMARKS_MSG"))

(in-package "ROS")
;;//! \htmlinclude Landmarks_Msg.msg.html


(defclass landmarks_msg::Landmarks_Msg
  :super ros::object
  :slots (_landmarks ))

(defmethod landmarks_msg::Landmarks_Msg
  (:init
   (&key
    ((:landmarks __landmarks) (let (r) (dotimes (i 0) (push (instance landmarks_msg::Landmark_Msg :init) r)) r))
    )
   (send-super :init)
   (setq _landmarks __landmarks)
   self)
  (:landmarks
   (&rest __landmarks)
   (if (keywordp (car __landmarks))
       (send* _landmarks __landmarks)
     (progn
       (if __landmarks (setq _landmarks (car __landmarks)))
       _landmarks)))
  (:serialization-length
   ()
   (+
    ;; landmarks_msg/Landmark_Msg[] _landmarks
    (apply #'+ (send-all _landmarks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; landmarks_msg/Landmark_Msg[] _landmarks
     (write-long (length _landmarks) s)
     (dolist (elem _landmarks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; landmarks_msg/Landmark_Msg[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance landmarks_msg::Landmark_Msg :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get landmarks_msg::Landmarks_Msg :md5sum-) "335d6c6a488c5701dc0c2d119f5c2eb2")
(setf (get landmarks_msg::Landmarks_Msg :datatype-) "landmarks_msg/Landmarks_Msg")
(setf (get landmarks_msg::Landmarks_Msg :definition-)
      "Landmark_Msg[] landmarks


================================================================================
MSG: landmarks_msg/Landmark_Msg
uint64 label
float32 x
float32 y
float32 s_x 
float32 s_y 





")



(provide :landmarks_msg/Landmarks_Msg "335d6c6a488c5701dc0c2d119f5c2eb2")


