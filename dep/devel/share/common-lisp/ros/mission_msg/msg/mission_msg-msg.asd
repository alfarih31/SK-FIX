
(cl:in-package :asdf)

(defsystem "mission_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Mission" :depends-on ("_package_Mission"))
    (:file "_package_Mission" :depends-on ("_package"))
  ))