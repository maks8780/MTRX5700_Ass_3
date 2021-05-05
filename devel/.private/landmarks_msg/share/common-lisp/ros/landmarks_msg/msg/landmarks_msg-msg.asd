
(cl:in-package :asdf)

(defsystem "landmarks_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Landmark_Msg" :depends-on ("_package_Landmark_Msg"))
    (:file "_package_Landmark_Msg" :depends-on ("_package"))
    (:file "Landmarks_Msg" :depends-on ("_package_Landmarks_Msg"))
    (:file "_package_Landmarks_Msg" :depends-on ("_package"))
  ))