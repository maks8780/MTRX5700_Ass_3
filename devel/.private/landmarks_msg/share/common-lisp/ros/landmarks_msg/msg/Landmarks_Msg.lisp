; Auto-generated. Do not edit!


(cl:in-package landmarks_msg-msg)


;//! \htmlinclude Landmarks_Msg.msg.html

(cl:defclass <Landmarks_Msg> (roslisp-msg-protocol:ros-message)
  ((landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector landmarks_msg-msg:Landmark_Msg)
   :initform (cl:make-array 0 :element-type 'landmarks_msg-msg:Landmark_Msg :initial-element (cl:make-instance 'landmarks_msg-msg:Landmark_Msg))))
)

(cl:defclass Landmarks_Msg (<Landmarks_Msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmarks_Msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmarks_Msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name landmarks_msg-msg:<Landmarks_Msg> is deprecated: use landmarks_msg-msg:Landmarks_Msg instead.")))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <Landmarks_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:landmarks-val is deprecated.  Use landmarks_msg-msg:landmarks instead.")
  (landmarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmarks_Msg>) ostream)
  "Serializes a message object of type '<Landmarks_Msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmarks_Msg>) istream)
  "Deserializes a message object of type '<Landmarks_Msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'landmarks_msg-msg:Landmark_Msg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmarks_Msg>)))
  "Returns string type for a message object of type '<Landmarks_Msg>"
  "landmarks_msg/Landmarks_Msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmarks_Msg)))
  "Returns string type for a message object of type 'Landmarks_Msg"
  "landmarks_msg/Landmarks_Msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmarks_Msg>)))
  "Returns md5sum for a message object of type '<Landmarks_Msg>"
  "335d6c6a488c5701dc0c2d119f5c2eb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmarks_Msg)))
  "Returns md5sum for a message object of type 'Landmarks_Msg"
  "335d6c6a488c5701dc0c2d119f5c2eb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmarks_Msg>)))
  "Returns full string definition for message of type '<Landmarks_Msg>"
  (cl:format cl:nil "Landmark_Msg[] landmarks~%~%~%================================================================================~%MSG: landmarks_msg/Landmark_Msg~%uint64 label~%float32 x~%float32 y~%float32 s_x ~%float32 s_y ~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmarks_Msg)))
  "Returns full string definition for message of type 'Landmarks_Msg"
  (cl:format cl:nil "Landmark_Msg[] landmarks~%~%~%================================================================================~%MSG: landmarks_msg/Landmark_Msg~%uint64 label~%float32 x~%float32 y~%float32 s_x ~%float32 s_y ~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmarks_Msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmarks_Msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmarks_Msg
    (cl:cons ':landmarks (landmarks msg))
))
