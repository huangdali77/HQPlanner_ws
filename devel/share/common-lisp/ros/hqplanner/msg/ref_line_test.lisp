; Auto-generated. Do not edit!


(cl:in-package hqplanner-msg)


;//! \htmlinclude ref_line_test.msg.html

(cl:defclass <ref_line_test> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ref_line_test (<ref_line_test>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ref_line_test>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ref_line_test)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hqplanner-msg:<ref_line_test> is deprecated: use hqplanner-msg:ref_line_test instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <ref_line_test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hqplanner-msg:path-val is deprecated.  Use hqplanner-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ref_line_test>) ostream)
  "Serializes a message object of type '<ref_line_test>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ref_line_test>) istream)
  "Deserializes a message object of type '<ref_line_test>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ref_line_test>)))
  "Returns string type for a message object of type '<ref_line_test>"
  "hqplanner/ref_line_test")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ref_line_test)))
  "Returns string type for a message object of type 'ref_line_test"
  "hqplanner/ref_line_test")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ref_line_test>)))
  "Returns md5sum for a message object of type '<ref_line_test>"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ref_line_test)))
  "Returns md5sum for a message object of type 'ref_line_test"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ref_line_test>)))
  "Returns full string definition for message of type '<ref_line_test>"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ref_line_test)))
  "Returns full string definition for message of type 'ref_line_test"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ref_line_test>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ref_line_test>))
  "Converts a ROS message object to a list"
  (cl:list 'ref_line_test
    (cl:cons ':path (path msg))
))
