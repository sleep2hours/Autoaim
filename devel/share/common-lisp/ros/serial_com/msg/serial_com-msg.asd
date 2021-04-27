
(cl:in-package :asdf)

(defsystem "serial_com-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "comm" :depends-on ("_package_comm"))
    (:file "_package_comm" :depends-on ("_package"))
    (:file "parameters" :depends-on ("_package_parameters"))
    (:file "_package_parameters" :depends-on ("_package"))
  ))