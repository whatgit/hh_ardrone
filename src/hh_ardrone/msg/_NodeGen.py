"""autogenerated by genpy from hh_ardrone/NodeGen.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class NodeGen(genpy.Message):
  _md5sum = "341c6c552736db523bca0b1a46668e59"
  _type = "hh_ardrone/NodeGen"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32      tag_no		
float32     x
float32     y
float32     z
float32	    tag_x
float32	    tag_y

"""
  __slots__ = ['tag_no','x','y','z','tag_x','tag_y']
  _slot_types = ['uint32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tag_no,x,y,z,tag_x,tag_y

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NodeGen, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.tag_no is None:
        self.tag_no = 0
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.tag_x is None:
        self.tag_x = 0.
      if self.tag_y is None:
        self.tag_y = 0.
    else:
      self.tag_no = 0
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.tag_x = 0.
      self.tag_y = 0.

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
      buff.write(_struct_I5f.pack(_x.tag_no, _x.x, _x.y, _x.z, _x.tag_x, _x.tag_y))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.tag_no, _x.x, _x.y, _x.z, _x.tag_x, _x.tag_y,) = _struct_I5f.unpack(str[start:end])
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
      buff.write(_struct_I5f.pack(_x.tag_no, _x.x, _x.y, _x.z, _x.tag_x, _x.tag_y))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

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
      end += 24
      (_x.tag_no, _x.x, _x.y, _x.z, _x.tag_x, _x.tag_y,) = _struct_I5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_I5f = struct.Struct("<I5f")
