; Auto-generated. Do not edit!


(cl:in-package haws-msg)


;//! \htmlinclude Tags.msg.html

(cl:defclass <Tags> (roslisp-msg-protocol:ros-message)
  ((tags
    :reader tags
    :initarg :tags
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Tags (<Tags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name haws-msg:<Tags> is deprecated: use haws-msg:Tags instead.")))

(cl:ensure-generic-function 'tags-val :lambda-list '(m))
(cl:defmethod tags-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:tags-val is deprecated.  Use haws-msg:tags instead.")
  (tags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tags>) ostream)
  "Serializes a message object of type '<Tags>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tags))))
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
   (cl:slot-value msg 'tags))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tags>) istream)
  "Deserializes a message object of type '<Tags>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tags) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tags)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tags>)))
  "Returns string type for a message object of type '<Tags>"
  "haws/Tags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tags)))
  "Returns string type for a message object of type 'Tags"
  "haws/Tags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tags>)))
  "Returns md5sum for a message object of type '<Tags>"
  "6937f05c43a3e459f7b8798e0b6d335d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tags)))
  "Returns md5sum for a message object of type 'Tags"
  "6937f05c43a3e459f7b8798e0b6d335d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tags>)))
  "Returns full string definition for message of type '<Tags>"
  (cl:format cl:nil "float64[] tags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tags)))
  "Returns full string definition for message of type 'Tags"
  (cl:format cl:nil "float64[] tags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tags>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tags) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tags>))
  "Converts a ROS message object to a list"
  (cl:list 'Tags
    (cl:cons ':tags (tags msg))
))
