
(cl:in-package :asdf)

(defsystem "haws-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tags" :depends-on ("_package_Tags"))
    (:file "_package_Tags" :depends-on ("_package"))
    (:file "Conflict" :depends-on ("_package_Conflict"))
    (:file "_package_Conflict" :depends-on ("_package"))
    (:file "Warning_Levels" :depends-on ("_package_Warning_Levels"))
    (:file "_package_Warning_Levels" :depends-on ("_package"))
  ))