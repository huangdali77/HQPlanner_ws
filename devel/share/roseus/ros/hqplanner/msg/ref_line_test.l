;; Auto-generated. Do not edit!


(when (boundp 'hqplanner::ref_line_test)
  (if (not (find-package "HQPLANNER"))
    (make-package "HQPLANNER"))
  (shadow 'ref_line_test (find-package "HQPLANNER")))
(unless (find-package "HQPLANNER::REF_LINE_TEST")
  (make-package "HQPLANNER::REF_LINE_TEST"))

(in-package "ROS")
;;//! \htmlinclude ref_line_test.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hqplanner::ref_line_test
  :super ros::object
  :slots (_path ))

(defmethod hqplanner::ref_line_test
  (:init
   (&key
    ((:path __path) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get hqplanner::ref_line_test :md5sum-) "b58b29f4d3d5430fc9d5efc2f5262786")
(setf (get hqplanner::ref_line_test :datatype-) "hqplanner/ref_line_test")
(setf (get hqplanner::ref_line_test :definition-)
      "geometry_msgs/Point[] path

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hqplanner/ref_line_test "b58b29f4d3d5430fc9d5efc2f5262786")


