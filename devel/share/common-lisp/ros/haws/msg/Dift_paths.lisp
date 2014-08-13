; Auto-generated. Do not edit!


(cl:in-package haws-msg)


;//! \htmlinclude Dift_paths.msg.html

(cl:defclass <Dift_paths> (roslisp-msg-protocol:ros-message)
  ((path_v
    :reader path_v
    :initarg :path_v
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (path_w
    :reader path_w
    :initarg :path_w
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (path_c
    :reader path_c
    :initarg :path_c
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Dift_paths (<Dift_paths>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dift_paths>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dift_paths)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name haws-msg:<Dift_paths> is deprecated: use haws-msg:Dift_paths instead.")))

(cl:ensure-generic-function 'path_v-val :lambda-list '(m))
(cl:defmethod path_v-val ((m <Dift_paths>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:path_v-val is deprecated.  Use haws-msg:path_v instead.")
  (path_v m))

(cl:ensure-generic-function 'path_w-val :lambda-list '(m))
(cl:defmethod path_w-val ((m <Dift_paths>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:path_w-val is deprecated.  Use haws-msg:path_w instead.")
  (path_w m))

(cl:ensure-generic-function 'path_c-val :lambda-list '(m))
(cl:defmethod path_c-val ((m <Dift_paths>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:path_c-val is deprecated.  Use haws-msg:path_c instead.")
  (path_c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dift_paths>) ostream)
  "Serializes a message object of type '<Dift_paths>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'path_v))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'path_w))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'path_c))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dift_paths>) istream)
  "Deserializes a message object of type '<Dift_paths>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_v) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_v)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_w) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_w)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_c) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_c)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dift_paths>)))
  "Returns string type for a message object of type '<Dift_paths>"
  "haws/Dift_paths")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dift_paths)))
  "Returns string type for a message object of type 'Dift_paths"
  "haws/Dift_paths")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dift_paths>)))
  "Returns md5sum for a message object of type '<Dift_paths>"
  "39a8534fd21d56d92f54ca2308fa2db8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dift_paths)))
  "Returns md5sum for a message object of type 'Dift_paths"
  "39a8534fd21d56d92f54ca2308fa2db8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dift_paths>)))
  "Returns full string definition for message of type '<Dift_paths>"
  (cl:format cl:nil "float32[] path_v~%float32[] path_w~%float32[] path_c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dift_paths)))
  "Returns full string definition for message of type 'Dift_paths"
  (cl:format cl:nil "float32[] path_v~%float32[] path_w~%float32[] path_c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dift_paths>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_v) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_w) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_c) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dift_paths>))
  "Converts a ROS message object to a list"
  (cl:list 'Dift_paths
    (cl:cons ':path_v (path_v msg))
    (cl:cons ':path_w (path_w msg))
    (cl:cons ':path_c (path_c msg))
))
