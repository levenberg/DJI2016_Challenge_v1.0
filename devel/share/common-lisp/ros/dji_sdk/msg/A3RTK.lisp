; Auto-generated. Do not edit!


(cl:in-package dji_sdk-msg)


;//! \htmlinclude A3RTK.msg.html

(cl:defclass <A3RTK> (roslisp-msg-protocol:ros-message)
  ((date
    :reader date
    :initarg :date
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0)
   (longitude_RTK
    :reader longitude_RTK
    :initarg :longitude_RTK
    :type cl:float
    :initform 0.0)
   (latitude_RTK
    :reader latitude_RTK
    :initarg :latitude_RTK
    :type cl:float
    :initform 0.0)
   (height_above_sea_RTK
    :reader height_above_sea_RTK
    :initarg :height_above_sea_RTK
    :type cl:float
    :initform 0.0)
   (longitude_single
    :reader longitude_single
    :initarg :longitude_single
    :type cl:integer
    :initform 0)
   (latitude_single
    :reader latitude_single
    :initarg :latitude_single
    :type cl:integer
    :initform 0)
   (height_above_sea_single
    :reader height_above_sea_single
    :initarg :height_above_sea_single
    :type cl:integer
    :initform 0)
   (velocity_north
    :reader velocity_north
    :initarg :velocity_north
    :type cl:float
    :initform 0.0)
   (velocity_east
    :reader velocity_east
    :initarg :velocity_east
    :type cl:float
    :initform 0.0)
   (velocity_ground
    :reader velocity_ground
    :initarg :velocity_ground
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:fixnum
    :initform 0)
   (satellite_used_RTK
    :reader satellite_used_RTK
    :initarg :satellite_used_RTK
    :type cl:fixnum
    :initform 0)
   (satellite_used_single
    :reader satellite_used_single
    :initarg :satellite_used_single
    :type cl:fixnum
    :initform 0)
   (horizontal_dop
    :reader horizontal_dop
    :initarg :horizontal_dop
    :type cl:float
    :initform 0.0)
   (position_dop
    :reader position_dop
    :initarg :position_dop
    :type cl:float
    :initform 0.0)
   (position_flag
    :reader position_flag
    :initarg :position_flag
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (gps_state
    :reader gps_state
    :initarg :gps_state
    :type cl:fixnum
    :initform 0)
   (rtk_updated_flag
    :reader rtk_updated_flag
    :initarg :rtk_updated_flag
    :type cl:fixnum
    :initform 0))
)

(cl:defclass A3RTK (<A3RTK>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <A3RTK>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'A3RTK)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dji_sdk-msg:<A3RTK> is deprecated: use dji_sdk-msg:A3RTK instead.")))

(cl:ensure-generic-function 'date-val :lambda-list '(m))
(cl:defmethod date-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:date-val is deprecated.  Use dji_sdk-msg:date instead.")
  (date m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:time-val is deprecated.  Use dji_sdk-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'longitude_RTK-val :lambda-list '(m))
(cl:defmethod longitude_RTK-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:longitude_RTK-val is deprecated.  Use dji_sdk-msg:longitude_RTK instead.")
  (longitude_RTK m))

(cl:ensure-generic-function 'latitude_RTK-val :lambda-list '(m))
(cl:defmethod latitude_RTK-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:latitude_RTK-val is deprecated.  Use dji_sdk-msg:latitude_RTK instead.")
  (latitude_RTK m))

(cl:ensure-generic-function 'height_above_sea_RTK-val :lambda-list '(m))
(cl:defmethod height_above_sea_RTK-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:height_above_sea_RTK-val is deprecated.  Use dji_sdk-msg:height_above_sea_RTK instead.")
  (height_above_sea_RTK m))

(cl:ensure-generic-function 'longitude_single-val :lambda-list '(m))
(cl:defmethod longitude_single-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:longitude_single-val is deprecated.  Use dji_sdk-msg:longitude_single instead.")
  (longitude_single m))

(cl:ensure-generic-function 'latitude_single-val :lambda-list '(m))
(cl:defmethod latitude_single-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:latitude_single-val is deprecated.  Use dji_sdk-msg:latitude_single instead.")
  (latitude_single m))

(cl:ensure-generic-function 'height_above_sea_single-val :lambda-list '(m))
(cl:defmethod height_above_sea_single-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:height_above_sea_single-val is deprecated.  Use dji_sdk-msg:height_above_sea_single instead.")
  (height_above_sea_single m))

(cl:ensure-generic-function 'velocity_north-val :lambda-list '(m))
(cl:defmethod velocity_north-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:velocity_north-val is deprecated.  Use dji_sdk-msg:velocity_north instead.")
  (velocity_north m))

(cl:ensure-generic-function 'velocity_east-val :lambda-list '(m))
(cl:defmethod velocity_east-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:velocity_east-val is deprecated.  Use dji_sdk-msg:velocity_east instead.")
  (velocity_east m))

(cl:ensure-generic-function 'velocity_ground-val :lambda-list '(m))
(cl:defmethod velocity_ground-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:velocity_ground-val is deprecated.  Use dji_sdk-msg:velocity_ground instead.")
  (velocity_ground m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:yaw-val is deprecated.  Use dji_sdk-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'satellite_used_RTK-val :lambda-list '(m))
(cl:defmethod satellite_used_RTK-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:satellite_used_RTK-val is deprecated.  Use dji_sdk-msg:satellite_used_RTK instead.")
  (satellite_used_RTK m))

(cl:ensure-generic-function 'satellite_used_single-val :lambda-list '(m))
(cl:defmethod satellite_used_single-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:satellite_used_single-val is deprecated.  Use dji_sdk-msg:satellite_used_single instead.")
  (satellite_used_single m))

(cl:ensure-generic-function 'horizontal_dop-val :lambda-list '(m))
(cl:defmethod horizontal_dop-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:horizontal_dop-val is deprecated.  Use dji_sdk-msg:horizontal_dop instead.")
  (horizontal_dop m))

(cl:ensure-generic-function 'position_dop-val :lambda-list '(m))
(cl:defmethod position_dop-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:position_dop-val is deprecated.  Use dji_sdk-msg:position_dop instead.")
  (position_dop m))

(cl:ensure-generic-function 'position_flag-val :lambda-list '(m))
(cl:defmethod position_flag-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:position_flag-val is deprecated.  Use dji_sdk-msg:position_flag instead.")
  (position_flag m))

(cl:ensure-generic-function 'gps_state-val :lambda-list '(m))
(cl:defmethod gps_state-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:gps_state-val is deprecated.  Use dji_sdk-msg:gps_state instead.")
  (gps_state m))

(cl:ensure-generic-function 'rtk_updated_flag-val :lambda-list '(m))
(cl:defmethod rtk_updated_flag-val ((m <A3RTK>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dji_sdk-msg:rtk_updated_flag-val is deprecated.  Use dji_sdk-msg:rtk_updated_flag instead.")
  (rtk_updated_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <A3RTK>) ostream)
  "Serializes a message object of type '<A3RTK>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'date)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'date)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'date)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'date)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude_RTK))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude_RTK))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_above_sea_RTK))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'longitude_single)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'latitude_single)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height_above_sea_single)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_north))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_east))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_ground))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'yaw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'satellite_used_RTK)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'satellite_used_single)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'horizontal_dop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position_dop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'position_flag))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rtk_updated_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rtk_updated_flag)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <A3RTK>) istream)
  "Deserializes a message object of type '<A3RTK>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'date)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'date)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'date)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'date)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude_RTK) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude_RTK) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_above_sea_RTK) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'longitude_single) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'latitude_single) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height_above_sea_single) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_north) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_east) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_ground) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yaw) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'satellite_used_RTK)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'satellite_used_single)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_dop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_dop) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'position_flag) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'position_flag)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rtk_updated_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rtk_updated_flag)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<A3RTK>)))
  "Returns string type for a message object of type '<A3RTK>"
  "dji_sdk/A3RTK")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'A3RTK)))
  "Returns string type for a message object of type 'A3RTK"
  "dji_sdk/A3RTK")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<A3RTK>)))
  "Returns md5sum for a message object of type '<A3RTK>"
  "5767e522bb89a710642f4bf98407bfa5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'A3RTK)))
  "Returns md5sum for a message object of type 'A3RTK"
  "5767e522bb89a710642f4bf98407bfa5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<A3RTK>)))
  "Returns full string definition for message of type '<A3RTK>"
  (cl:format cl:nil "uint32 date~%uint32 time~%float64 longitude_RTK~%float64 latitude_RTK~%float32 height_above_sea_RTK~%int32 longitude_single~%int32 latitude_single~%int32 height_above_sea_single~%float32 velocity_north~%float32 velocity_east~%float32 velocity_ground~%int16 yaw #between baseline and south~%uint8 satellite_used_RTK~%uint8 satellite_used_single~%float32 horizontal_dop~%float32 position_dop~%uint8[6] position_flag #0 sigle point, 1 RTK, 2 fixed direction, 3 reserve~%uint16 gps_state~%uint16 rtk_updated_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'A3RTK)))
  "Returns full string definition for message of type 'A3RTK"
  (cl:format cl:nil "uint32 date~%uint32 time~%float64 longitude_RTK~%float64 latitude_RTK~%float32 height_above_sea_RTK~%int32 longitude_single~%int32 latitude_single~%int32 height_above_sea_single~%float32 velocity_north~%float32 velocity_east~%float32 velocity_ground~%int16 yaw #between baseline and south~%uint8 satellite_used_RTK~%uint8 satellite_used_single~%float32 horizontal_dop~%float32 position_dop~%uint8[6] position_flag #0 sigle point, 1 RTK, 2 fixed direction, 3 reserve~%uint16 gps_state~%uint16 rtk_updated_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <A3RTK>))
  (cl:+ 0
     4
     4
     8
     8
     4
     4
     4
     4
     4
     4
     4
     2
     1
     1
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position_flag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <A3RTK>))
  "Converts a ROS message object to a list"
  (cl:list 'A3RTK
    (cl:cons ':date (date msg))
    (cl:cons ':time (time msg))
    (cl:cons ':longitude_RTK (longitude_RTK msg))
    (cl:cons ':latitude_RTK (latitude_RTK msg))
    (cl:cons ':height_above_sea_RTK (height_above_sea_RTK msg))
    (cl:cons ':longitude_single (longitude_single msg))
    (cl:cons ':latitude_single (latitude_single msg))
    (cl:cons ':height_above_sea_single (height_above_sea_single msg))
    (cl:cons ':velocity_north (velocity_north msg))
    (cl:cons ':velocity_east (velocity_east msg))
    (cl:cons ':velocity_ground (velocity_ground msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':satellite_used_RTK (satellite_used_RTK msg))
    (cl:cons ':satellite_used_single (satellite_used_single msg))
    (cl:cons ':horizontal_dop (horizontal_dop msg))
    (cl:cons ':position_dop (position_dop msg))
    (cl:cons ':position_flag (position_flag msg))
    (cl:cons ':gps_state (gps_state msg))
    (cl:cons ':rtk_updated_flag (rtk_updated_flag msg))
))
