"""autogenerated by genpy from ubot_dmp/DMPGoal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class DMPGoal(genpy.Message):
  _md5sum = "09be01e5815b7925cf831dc14edf1c47"
  _type = "ubot_dmp/DMPGoal"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# define the goalmessage

# Must be all twelve joints.
float32[] goal_pos 

# selection bitmask: only joint whose bit are 1 will are commanded
int32 bitmask

# time parameter tau as specified in 2013 schaal paper. 1 means normal time (what DMP was recorded at)
float32 tau

# Where to find the DMP in the library. This should match the file structure of the pickles folder
string dmp_id
string shelf
string section

"""
  __slots__ = ['goal_pos','bitmask','tau','dmp_id','shelf','section']
  _slot_types = ['float32[]','int32','float32','string','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       goal_pos,bitmask,tau,dmp_id,shelf,section

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DMPGoal, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.goal_pos is None:
        self.goal_pos = []
      if self.bitmask is None:
        self.bitmask = 0
      if self.tau is None:
        self.tau = 0.
      if self.dmp_id is None:
        self.dmp_id = ''
      if self.shelf is None:
        self.shelf = ''
      if self.section is None:
        self.section = ''
    else:
      self.goal_pos = []
      self.bitmask = 0
      self.tau = 0.
      self.dmp_id = ''
      self.shelf = ''
      self.section = ''

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
      length = len(self.goal_pos)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.goal_pos))
      _x = self
      buff.write(_struct_if.pack(_x.bitmask, _x.tau))
      _x = self.dmp_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.shelf
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.section
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.goal_pos = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.bitmask, _x.tau,) = _struct_if.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.dmp_id = str[start:end].decode('utf-8')
      else:
        self.dmp_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.shelf = str[start:end].decode('utf-8')
      else:
        self.shelf = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.section = str[start:end].decode('utf-8')
      else:
        self.section = str[start:end]
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
      length = len(self.goal_pos)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.goal_pos.tostring())
      _x = self
      buff.write(_struct_if.pack(_x.bitmask, _x.tau))
      _x = self.dmp_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.shelf
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.section
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.goal_pos = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 8
      (_x.bitmask, _x.tau,) = _struct_if.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.dmp_id = str[start:end].decode('utf-8')
      else:
        self.dmp_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.shelf = str[start:end].decode('utf-8')
      else:
        self.shelf = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.section = str[start:end].decode('utf-8')
      else:
        self.section = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_if = struct.Struct("<if")
