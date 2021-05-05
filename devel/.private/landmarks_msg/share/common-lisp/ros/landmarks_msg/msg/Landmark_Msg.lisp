; Auto-generated. Do not edit!


(cl:in-package landmarks_msg-msg)


;//! \htmlinclude Landmark_Msg.msg.html

(cl:defclass <Landmark_Msg> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (s_x
    :reader s_x
    :initarg :s_x
    :type cl:float
    :initform 0.0)
   (s_y
    :reader s_y
    :initarg :s_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Landmark_Msg (<Landmark_Msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmark_Msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmark_Msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name landmarks_msg-msg:<Landmark_Msg> is deprecated: use landmarks_msg-msg:Landmark_Msg instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <Landmark_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:label-val is deprecated.  Use landmarks_msg-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Landmark_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:x-val is deprecated.  Use landmarks_msg-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Landmark_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:y-val is deprecated.  Use landmarks_msg-msg:y instead.")
  (y m))

(cl:ensure-generic-function 's_x-val :lambda-list '(m))
(cl:defmethod s_x-val ((m <Landmark_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:s_x-val is deprecated.  Use landmarks_msg-msg:s_x instead.")
  (s_x m))

(cl:ensure-generic-function 's_y-val :lambda-list '(m))
(cl:defmethod s_y-val ((m <Landmark_Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader landmarks_msg-msg:s_y-val is deprecated.  Use landmarks_msg-msg:s_y instead.")
  (s_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmark_Msg>) ostream)
  "Serializes a message object of type '<Landmark_Msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'label)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'label)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmark_Msg>) istream)
  "Deserializes a message object of type '<Landmark_Msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'label)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmark_Msg>)))
  "Returns string type for a message object of type '<Landmark_Msg>"
  "landmarks_msg/Landmark_Msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark_Msg)))
  "Returns string type for a message object of type 'Landmark_Msg"
  "landmarks_msg/Landmark_Msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmark_Msg>)))
  "Returns md5sum for a message object of type '<Landmark_Msg>"
  "37586ccc5f4ed3186510933e2fe487e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmark_Msg)))
  "Returns md5sum for a message object of type 'Landmark_Msg"
  "37586ccc5f4ed3186510933e2fe487e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmark_Msg>)))
  "Returns full string definition for message of type '<Landmark_Msg>"
  (cl:format cl:nil "uint64 label~%float32 x~%float32 y~%float32 s_x ~%float32 s_y ~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmark_Msg)))
  "Returns full string definition for message of type 'Landmark_Msg"
  (cl:format cl:nil "uint64 label~%float32 x~%float32 y~%float32 s_x ~%float32 s_y ~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmark_Msg>))
  (cl:+ 0
     8
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmark_Msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmark_Msg
    (cl:cons ':label (label msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':s_x (s_x msg))
    (cl:cons ':s_y (s_y msg))
))
