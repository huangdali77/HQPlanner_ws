
(cl:in-package :asdf)

(defsystem "hqplanner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ref_line_test" :depends-on ("_package_ref_line_test"))
    (:file "_package_ref_line_test" :depends-on ("_package"))
  ))