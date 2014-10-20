; Auto-generated. Do not edit!


(cl:in-package haws-msg)


;//! \htmlinclude Warning_Levels.msg.html

(cl:defclass <Warning_Levels> (roslisp-msg-protocol:ros-message)
  ((warning_levels
    :reader warning_levels
    :initarg :warning_levels
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Warning_Levels (<Warning_Levels>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Warning_Levels>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Warning_Levels)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name haws-msg:<Warning_Levels> is deprecated: use haws-msg:Warning_Levels instead.")))

(cl:ensure-generic-function 'warning_levels-val :lambda-list '(m))
(cl:defmethod warning_levels-val ((m <Warning_Levels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:warning_levels-val is deprecated.  Use haws-msg:warning_levels instead.")
  (warning_levels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Warning_Levels>) ostream)
  "Serializes a message object of type '<Warning_Levels>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'warning_levels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'warning_levels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Warning_Levels>) istream)
  "Deserializes a message object of type '<Warning_Levels>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'warning_levels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'warning_levels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Warning_Levels>)))
  "Returns string type for a message object of type '<Warning_Levels>"
  "haws/Warning_Levels")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Warning_Levels)))
  "Returns string type for a message object of type 'Warning_Levels"
  "haws/Warning_Levels")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Warning_Levels>)))
  "Returns md5sum for a message object of type '<Warning_Levels>"
  "d4c5c847950ed195bbc9a4965e3b91f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Warning_Levels)))
  "Returns md5sum for a message object of type 'Warning_Levels"
  "d4c5c847950ed195bbc9a4965e3b91f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Warning_Levels>)))
  "Returns full string definition for message of type '<Warning_Levels>"
  (cl:format cl:nil "float64[] warning_levels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Warning_Levels)))
  "Returns full string definition for message of type 'Warning_Levels"
  (cl:format cl:nil "float64[] warning_levels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Warning_Levels>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'warning_levels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Warning_Levels>))
  "Converts a ROS message object to a list"
  (cl:list 'Warning_Levels
    (cl:cons ':warning_levels (warning_levels msg))
))
