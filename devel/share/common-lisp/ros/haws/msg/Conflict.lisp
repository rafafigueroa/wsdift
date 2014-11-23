; Auto-generated. Do not edit!


(cl:in-package haws-msg)


;//! \htmlinclude Conflict.msg.html

(cl:defclass <Conflict> (roslisp-msg-protocol:ros-message)
  ((avoid_activated
    :reader avoid_activated
    :initarg :avoid_activated
    :type cl:boolean
    :initform cl:nil)
   (tc
    :reader tc
    :initarg :tc
    :type cl:float
    :initform 0.0)
   (gamma
    :reader gamma
    :initarg :gamma
    :type cl:float
    :initform 0.0))
)

(cl:defclass Conflict (<Conflict>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Conflict>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Conflict)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name haws-msg:<Conflict> is deprecated: use haws-msg:Conflict instead.")))

(cl:ensure-generic-function 'avoid_activated-val :lambda-list '(m))
(cl:defmethod avoid_activated-val ((m <Conflict>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:avoid_activated-val is deprecated.  Use haws-msg:avoid_activated instead.")
  (avoid_activated m))

(cl:ensure-generic-function 'tc-val :lambda-list '(m))
(cl:defmethod tc-val ((m <Conflict>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:tc-val is deprecated.  Use haws-msg:tc instead.")
  (tc m))

(cl:ensure-generic-function 'gamma-val :lambda-list '(m))
(cl:defmethod gamma-val ((m <Conflict>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader haws-msg:gamma-val is deprecated.  Use haws-msg:gamma instead.")
  (gamma m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Conflict>) ostream)
  "Serializes a message object of type '<Conflict>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'avoid_activated) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Conflict>) istream)
  "Deserializes a message object of type '<Conflict>"
    (cl:setf (cl:slot-value msg 'avoid_activated) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tc) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gamma) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Conflict>)))
  "Returns string type for a message object of type '<Conflict>"
  "haws/Conflict")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Conflict)))
  "Returns string type for a message object of type 'Conflict"
  "haws/Conflict")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Conflict>)))
  "Returns md5sum for a message object of type '<Conflict>"
  "01359ce202f254731c57b778944aa8d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Conflict)))
  "Returns md5sum for a message object of type 'Conflict"
  "01359ce202f254731c57b778944aa8d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Conflict>)))
  "Returns full string definition for message of type '<Conflict>"
  (cl:format cl:nil "bool avoid_activated~%float64 tc~%float64 gamma~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Conflict)))
  "Returns full string definition for message of type 'Conflict"
  (cl:format cl:nil "bool avoid_activated~%float64 tc~%float64 gamma~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Conflict>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Conflict>))
  "Converts a ROS message object to a list"
  (cl:list 'Conflict
    (cl:cons ':avoid_activated (avoid_activated msg))
    (cl:cons ':tc (tc msg))
    (cl:cons ':gamma (gamma msg))
))
