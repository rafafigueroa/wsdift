; Auto-generated. Do not edit!


(cl:in-package haws-msg)


;//! \htmlinclude Tags.msg.html

(cl:defclass <Tags> (roslisp-msg-protocol:ros-message)
  ((do_nothing
    :reader do_nothing
    :initarg :do_nothing
    :type cl:fixnum
    :initform 0)
   (do_inputs
    :reader do_inputs
    :initarg :do_inputs
    :type cl:fixnum
    :initform 0)
   (do_current
    :reader do_current
    :initarg :do_current
    :type cl:fixnum
    :initform 0)
   (path_current
    :reader path_current
    :initarg :path_current
    :type (cl:vector haws-msg:Pose)
   :initform (cl:make-array 0 :element-type 'haws-msg:Pose :initial-element (cl:make-instance 'haws-msg:Pose))))
)

(cl:defclass Tags (<Tags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name haws-msg:<Tags> is deprecated: use haws-msg:Tags instead.")))

(cl:ensure-generic-function 'do_nothing-val :lambda-list '(m))
(cl:defmethod do_nothing-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:do_nothing-val is deprecated.  Use haws-msg:do_nothing instead.")
  (do_nothing m))

(cl:ensure-generic-function 'do_inputs-val :lambda-list '(m))
(cl:defmethod do_inputs-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:do_inputs-val is deprecated.  Use haws-msg:do_inputs instead.")
  (do_inputs m))

(cl:ensure-generic-function 'do_current-val :lambda-list '(m))
(cl:defmethod do_current-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:do_current-val is deprecated.  Use haws-msg:do_current instead.")
  (do_current m))

(cl:ensure-generic-function 'path_current-val :lambda-list '(m))
(cl:defmethod path_current-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:path_current-val is deprecated.  Use haws-msg:path_current instead.")
  (path_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tags>) ostream)
  "Serializes a message object of type '<Tags>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_nothing)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_inputs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_current)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path_current))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tags>) istream)
  "Deserializes a message object of type '<Tags>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_nothing)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_inputs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_current)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_current) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_current)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'haws-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  "6d844e42d2aea2ef03da0c40d877b30f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tags)))
  "Returns md5sum for a message object of type 'Tags"
  "6d844e42d2aea2ef03da0c40d877b30f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tags>)))
  "Returns full string definition for message of type '<Tags>"
  (cl:format cl:nil "uint8 do_nothing~%uint8 do_inputs~%uint8 do_current~%haws/Pose[] path_current~%	~%~%================================================================================~%MSG: haws/Pose~%float32 x~%float32 y~%float32 theta~%~%float32 linear_velocity~%float32 angular_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tags)))
  "Returns full string definition for message of type 'Tags"
  (cl:format cl:nil "uint8 do_nothing~%uint8 do_inputs~%uint8 do_current~%haws/Pose[] path_current~%	~%~%================================================================================~%MSG: haws/Pose~%float32 x~%float32 y~%float32 theta~%~%float32 linear_velocity~%float32 angular_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tags>))
  (cl:+ 0
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tags>))
  "Converts a ROS message object to a list"
  (cl:list 'Tags
    (cl:cons ':do_nothing (do_nothing msg))
    (cl:cons ':do_inputs (do_inputs msg))
    (cl:cons ':do_current (do_current msg))
    (cl:cons ':path_current (path_current msg))
))
