# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dji_sdk/A3GPS.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class A3GPS(genpy.Message):
  _md5sum = "bd0ebd44de0d17f8448fe67c3e9af79d"
  _type = "dji_sdk/A3GPS"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 date #GPS date
uint32 time #GPS time
int32 longitude #unit in degree*10^7
int32 latitude  #unit in degree*10^7
int32 height_above_sea #unit in mm 
float32 velocity_north #unit in cm/s
float32 velocity_east #unit in cm/s
float32 velocity_ground #unit in cm/s
float32 horizontal_dop
float32 position_dop 
float32 gps_fix
float32 vertical_position_accuracy
float32 horizontal_position_accuracy
float32 velocity_accuracy
uint32 gps_satellite_used
uint32 glonass_satellite_used
uint16 total_satellite_used
uint16 gps_state 
"""
  __slots__ = ['date','time','longitude','latitude','height_above_sea','velocity_north','velocity_east','velocity_ground','horizontal_dop','position_dop','gps_fix','vertical_position_accuracy','horizontal_position_accuracy','velocity_accuracy','gps_satellite_used','glonass_satellite_used','total_satellite_used','gps_state']
  _slot_types = ['uint32','uint32','int32','int32','int32','float32','float32','float32','float32','float32','float32','float32','float32','float32','uint32','uint32','uint16','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       date,time,longitude,latitude,height_above_sea,velocity_north,velocity_east,velocity_ground,horizontal_dop,position_dop,gps_fix,vertical_position_accuracy,horizontal_position_accuracy,velocity_accuracy,gps_satellite_used,glonass_satellite_used,total_satellite_used,gps_state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(A3GPS, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.date is None:
        self.date = 0
      if self.time is None:
        self.time = 0
      if self.longitude is None:
        self.longitude = 0
      if self.latitude is None:
        self.latitude = 0
      if self.height_above_sea is None:
        self.height_above_sea = 0
      if self.velocity_north is None:
        self.velocity_north = 0.
      if self.velocity_east is None:
        self.velocity_east = 0.
      if self.velocity_ground is None:
        self.velocity_ground = 0.
      if self.horizontal_dop is None:
        self.horizontal_dop = 0.
      if self.position_dop is None:
        self.position_dop = 0.
      if self.gps_fix is None:
        self.gps_fix = 0.
      if self.vertical_position_accuracy is None:
        self.vertical_position_accuracy = 0.
      if self.horizontal_position_accuracy is None:
        self.horizontal_position_accuracy = 0.
      if self.velocity_accuracy is None:
        self.velocity_accuracy = 0.
      if self.gps_satellite_used is None:
        self.gps_satellite_used = 0
      if self.glonass_satellite_used is None:
        self.glonass_satellite_used = 0
      if self.total_satellite_used is None:
        self.total_satellite_used = 0
      if self.gps_state is None:
        self.gps_state = 0
    else:
      self.date = 0
      self.time = 0
      self.longitude = 0
      self.latitude = 0
      self.height_above_sea = 0
      self.velocity_north = 0.
      self.velocity_east = 0.
      self.velocity_ground = 0.
      self.horizontal_dop = 0.
      self.position_dop = 0.
      self.gps_fix = 0.
      self.vertical_position_accuracy = 0.
      self.horizontal_position_accuracy = 0.
      self.velocity_accuracy = 0.
      self.gps_satellite_used = 0
      self.glonass_satellite_used = 0
      self.total_satellite_used = 0
      self.gps_state = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_2I3i9f2I2H.pack(_x.date, _x.time, _x.longitude, _x.latitude, _x.height_above_sea, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.horizontal_dop, _x.position_dop, _x.gps_fix, _x.vertical_position_accuracy, _x.horizontal_position_accuracy, _x.velocity_accuracy, _x.gps_satellite_used, _x.glonass_satellite_used, _x.total_satellite_used, _x.gps_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 68
      (_x.date, _x.time, _x.longitude, _x.latitude, _x.height_above_sea, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.horizontal_dop, _x.position_dop, _x.gps_fix, _x.vertical_position_accuracy, _x.horizontal_position_accuracy, _x.velocity_accuracy, _x.gps_satellite_used, _x.glonass_satellite_used, _x.total_satellite_used, _x.gps_state,) = _struct_2I3i9f2I2H.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_2I3i9f2I2H.pack(_x.date, _x.time, _x.longitude, _x.latitude, _x.height_above_sea, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.horizontal_dop, _x.position_dop, _x.gps_fix, _x.vertical_position_accuracy, _x.horizontal_position_accuracy, _x.velocity_accuracy, _x.gps_satellite_used, _x.glonass_satellite_used, _x.total_satellite_used, _x.gps_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 68
      (_x.date, _x.time, _x.longitude, _x.latitude, _x.height_above_sea, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.horizontal_dop, _x.position_dop, _x.gps_fix, _x.vertical_position_accuracy, _x.horizontal_position_accuracy, _x.velocity_accuracy, _x.gps_satellite_used, _x.glonass_satellite_used, _x.total_satellite_used, _x.gps_state,) = _struct_2I3i9f2I2H.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I3i9f2I2H = struct.Struct("<2I3i9f2I2H")
