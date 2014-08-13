
(cl:in-package :asdf)

(defsystem "haws-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tags" :depends-on ("_package_Tags"))
    (:file "_package_Tags" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
  ))