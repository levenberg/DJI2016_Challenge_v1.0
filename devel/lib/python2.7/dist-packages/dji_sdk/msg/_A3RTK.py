# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dji_sdk/A3RTK.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class A3RTK(genpy.Message):
  _md5sum = "5767e522bb89a710642f4bf98407bfa5"
  _type = "dji_sdk/A3RTK"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 date
uint32 time
float64 longitude_RTK
float64 latitude_RTK
float32 height_above_sea_RTK
int32 longitude_single
int32 latitude_single
int32 height_above_sea_single
float32 velocity_north
float32 velocity_east
float32 velocity_ground
int16 yaw #between baseline and south
uint8 satellite_used_RTK
uint8 satellite_used_single
float32 horizontal_dop
float32 position_dop
uint8[6] position_flag #0 sigle point, 1 RTK, 2 fixed direction, 3 reserve
uint16 gps_state
uint16 rtk_updated_flag
"""
  __slots__ = ['date','time','longitude_RTK','latitude_RTK','height_above_sea_RTK','longitude_single','latitude_single','height_above_sea_single','velocity_north','velocity_east','velocity_ground','yaw','satellite_used_RTK','satellite_used_single','horizontal_dop','position_dop','position_flag','gps_state','rtk_updated_flag']
  _slot_types = ['uint32','uint32','float64','float64','float32','int32','int32','int32','float32','float32','float32','int16','uint8','uint8','float32','float32','uint8[6]','uint16','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       date,time,longitude_RTK,latitude_RTK,height_above_sea_RTK,longitude_single,latitude_single,height_above_sea_single,velocity_north,velocity_east,velocity_ground,yaw,satellite_used_RTK,satellite_used_single,horizontal_dop,position_dop,position_flag,gps_state,rtk_updated_flag

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(A3RTK, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.date is None:
        self.date = 0
      if self.time is None:
        self.time = 0
      if self.longitude_RTK is None:
        self.longitude_RTK = 0.
      if self.latitude_RTK is None:
        self.latitude_RTK = 0.
      if self.height_above_sea_RTK is None:
        self.height_above_sea_RTK = 0.
      if self.longitude_single is None:
        self.longitude_single = 0
      if self.latitude_single is None:
        self.latitude_single = 0
      if self.height_above_sea_single is None:
        self.height_above_sea_single = 0
      if self.velocity_north is None:
        self.velocity_north = 0.
      if self.velocity_east is None:
        self.velocity_east = 0.
      if self.velocity_ground is None:
        self.velocity_ground = 0.
      if self.yaw is None:
        self.yaw = 0
      if self.satellite_used_RTK is None:
        self.satellite_used_RTK = 0
      if self.satellite_used_single is None:
        self.satellite_used_single = 0
      if self.horizontal_dop is None:
        self.horizontal_dop = 0.
      if self.position_dop is None:
        self.position_dop = 0.
      if self.position_flag is None:
        self.position_flag = chr(0)*6
      if self.gps_state is None:
        self.gps_state = 0
      if self.rtk_updated_flag is None:
        self.rtk_updated_flag = 0
    else:
      self.date = 0
      self.time = 0
      self.longitude_RTK = 0.
      self.latitude_RTK = 0.
      self.height_above_sea_RTK = 0.
      self.longitude_single = 0
      self.latitude_single = 0
      self.height_above_sea_single = 0
      self.velocity_north = 0.
      self.velocity_east = 0.
      self.velocity_ground = 0.
      self.yaw = 0
      self.satellite_used_RTK = 0
      self.satellite_used_single = 0
      self.horizontal_dop = 0.
      self.position_dop = 0.
      self.position_flag = chr(0)*6
      self.gps_state = 0
      self.rtk_updated_flag = 0

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
      buff.write(_struct_2I2df3i3fh2B2f.pack(_x.date, _x.time, _x.longitude_RTK, _x.latitude_RTK, _x.height_above_sea_RTK, _x.longitude_single, _x.latitude_single, _x.height_above_sea_single, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.yaw, _x.satellite_used_RTK, _x.satellite_used_single, _x.horizontal_dop, _x.position_dop))
      _x = self.position_flag
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_6B.pack(*_x))
      else:
        buff.write(_struct_6s.pack(_x))
      _x = self
      buff.write(_struct_2H.pack(_x.gps_state, _x.rtk_updated_flag))
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
      end += 64
      (_x.date, _x.time, _x.longitude_RTK, _x.latitude_RTK, _x.height_above_sea_RTK, _x.longitude_single, _x.latitude_single, _x.height_above_sea_single, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.yaw, _x.satellite_used_RTK, _x.satellite_used_single, _x.horizontal_dop, _x.position_dop,) = _struct_2I2df3i3fh2B2f.unpack(str[start:end])
      start = end
      end += 6
      self.position_flag = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.gps_state, _x.rtk_updated_flag,) = _struct_2H.unpack(str[start:end])
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
      buff.write(_struct_2I2df3i3fh2B2f.pack(_x.date, _x.time, _x.longitude_RTK, _x.latitude_RTK, _x.height_above_sea_RTK, _x.longitude_single, _x.latitude_single, _x.height_above_sea_single, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.yaw, _x.satellite_used_RTK, _x.satellite_used_single, _x.horizontal_dop, _x.position_dop))
      _x = self.position_flag
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_6B.pack(*_x))
      else:
        buff.write(_struct_6s.pack(_x))
      _x = self
      buff.write(_struct_2H.pack(_x.gps_state, _x.rtk_updated_flag))
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
      end += 64
      (_x.date, _x.time, _x.longitude_RTK, _x.latitude_RTK, _x.height_above_sea_RTK, _x.longitude_single, _x.latitude_single, _x.height_above_sea_single, _x.velocity_north, _x.velocity_east, _x.velocity_ground, _x.yaw, _x.satellite_used_RTK, _x.satellite_used_single, _x.horizontal_dop, _x.position_dop,) = _struct_2I2df3i3fh2B2f.unpack(str[start:end])
      start = end
      end += 6
      self.position_flag = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.gps_state, _x.rtk_updated_flag,) = _struct_2H.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2H = struct.Struct("<2H")
_struct_6B = struct.Struct("<6B")
_struct_6s = struct.Struct("<6s")
_struct_2I2df3i3fh2B2f = struct.Struct("<2I2df3i3fh2B2f")