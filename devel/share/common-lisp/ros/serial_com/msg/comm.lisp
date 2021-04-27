; Auto-generated. Do not edit!


(cl:in-package serial_com-msg)


;//! \htmlinclude comm.msg.html

(cl:defclass <comm> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass comm (<comm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <comm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'comm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_com-msg:<comm> is deprecated: use serial_com-msg:comm instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_com-msg:x-val is deprecated.  Use serial_com-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_com-msg:y-val is deprecated.  Use serial_com-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_com-msg:z-val is deprecated.  Use serial_com-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_com-msg:status-val is deprecated.  Use serial_com-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <comm>) ostream)
  "Serializes a message object of type '<comm>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <comm>) istream)
  "Deserializes a message object of type '<comm>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<comm>)))
  "Returns string type for a message object of type '<comm>"
  "serial_com/comm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'comm)))
  "Returns string type for a message object of type 'comm"
  "serial_com/comm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<comm>)))
  "Returns md5sum for a message object of type '<comm>"
  "833ea5f5972422df37fbfb2214b935ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'comm)))
  "Returns md5sum for a message object of type 'comm"
  "833ea5f5972422df37fbfb2214b935ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<comm>)))
  "Returns full string definition for message of type '<comm>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'comm)))
  "Returns full string definition for message of type 'comm"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%uint8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <comm>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <comm>))
  "Converts a ROS message object to a list"
  (cl:list 'comm
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':status (status msg))
))
