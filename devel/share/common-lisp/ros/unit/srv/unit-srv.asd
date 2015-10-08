
(cl:in-package :asdf)

(defsystem "unit-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "for_double" :depends-on ("_package_for_double"))
    (:file "_package_for_double" :depends-on ("_package"))
    (:file "from_robot" :depends-on ("_package_from_robot"))
    (:file "_package_from_robot" :depends-on ("_package"))
  ))