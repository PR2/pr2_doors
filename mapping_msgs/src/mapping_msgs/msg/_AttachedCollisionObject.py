"""autogenerated by genmsg_py from AttachedCollisionObject.msg. Do not edit."""
import roslib.message
import struct

import mapping_msgs.msg
import geometry_msgs.msg
import geometric_shapes_msgs.msg
import std_msgs.msg

class AttachedCollisionObject(roslib.message.Message):
  _md5sum = "58c7f119e35988da1dbd450c682308ed"
  _type = "mapping_msgs/AttachedCollisionObject"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# The CollisionObject will be attached with a fixed joint to this link
# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation 
# is set to REMOVE will remove all attached bodies attached to any object
string link_name

#Reserved for indicating that all attached objects should be removed
string REMOVE_ALL_ATTACHED_OBJECTS = "all"

#This contains the actual shapes and poses for the CollisionObject
#to be attached to the link
#If action is remove and no object.id is set, all objects
#attached to the link indicated by link_name will be removed
CollisionObject object

# The set of links that the attached objects are allowed to touch
# by default - the link_name is included by default
string[] touch_links

================================================================================
MSG: mapping_msgs/CollisionObject
# a header, used for interpreting the poses
Header header

# the id of the object
string id

#This contains what is to be done with the object
CollisionObjectOperation operation

#the shapes associated with the object
geometric_shapes_msgs/Shape[] shapes

#the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: mapping_msgs/CollisionObjectOperation
#Puts the object into the environment
#or updates the object if already added
byte ADD=0

#Removes the object from the environment entirely
byte REMOVE=1

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes an attached object, detaches from the attached link
#But adds back in as regular object
byte DETACH_AND_ADD_AS_OBJECT=2

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes current object in the environment and removes it as
#a regular object
byte ATTACH_AND_REMOVE_AS_OBJECT=3

# Byte code for operation
byte operation

================================================================================
MSG: geometric_shapes_msgs/Shape
byte SPHERE=0
byte BOX=1
byte CYLINDER=2
byte MESH=3

byte type


#### define sphere, box, cylinder ####
# the origin of each shape is considered at the shape's center

# for sphere
# radius := dimensions[0]

# for cylinder
# radius := dimensions[0]
# length := dimensions[1]
# the length is along the Z axis

# for box
# size_x := dimensions[0]
# size_y := dimensions[1]
# size_z := dimensions[2]
float64[] dimensions


#### define mesh ####

# list of triangles; triangle k is defined by tre vertices located
# at indices triangles[3k], triangles[3k+1], triangles[3k+2]
int32[] triangles
geometry_msgs/Point[] vertices

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  # Pseudo-constants
  REMOVE_ALL_ATTACHED_OBJECTS = r'"all"'

  __slots__ = ['link_name','object','touch_links']
  _slot_types = ['string','mapping_msgs/CollisionObject','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       link_name,object,touch_links
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(AttachedCollisionObject, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.link_name is None:
        self.link_name = ''
      if self.object is None:
        self.object = mapping_msgs.msg.CollisionObject()
      if self.touch_links is None:
        self.touch_links = []
    else:
      self.link_name = ''
      self.object = mapping_msgs.msg.CollisionObject()
      self.touch_links = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self.link_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.object.header.seq, _x.object.header.stamp.secs, _x.object.header.stamp.nsecs))
      _x = self.object.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.object.id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_b.pack(self.object.operation.operation))
      length = len(self.object.shapes)
      buff.write(_struct_I.pack(length))
      for val1 in self.object.shapes:
        buff.write(_struct_b.pack(val1.type))
        length = len(val1.dimensions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.pack(pattern, *val1.dimensions))
        length = len(val1.triangles)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.pack(pattern, *val1.triangles))
        length = len(val1.vertices)
        buff.write(_struct_I.pack(length))
        for val2 in val1.vertices:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.object.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.object.poses:
        _v1 = val1.position
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v2 = val1.orientation
        _x = _v2
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.touch_links)
      buff.write(_struct_I.pack(length))
      for val1 in self.touch_links:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.object is None:
        self.object = mapping_msgs.msg.CollisionObject()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.link_name = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.object.header.seq, _x.object.header.stamp.secs, _x.object.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.object.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.object.id = str[start:end]
      start = end
      end += 1
      (self.object.operation.operation,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object.shapes = []
      for i in range(0, length):
        val1 = geometric_shapes_msgs.msg.Shape()
        start = end
        end += 1
        (val1.type,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.dimensions = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        val1.triangles = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.vertices = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.vertices.append(val2)
        self.object.shapes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v3 = val1.position
        _x = _v3
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v4 = val1.orientation
        _x = _v4
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.object.poses.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.touch_links = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.touch_links.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self.link_name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.object.header.seq, _x.object.header.stamp.secs, _x.object.header.stamp.nsecs))
      _x = self.object.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.object.id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_b.pack(self.object.operation.operation))
      length = len(self.object.shapes)
      buff.write(_struct_I.pack(length))
      for val1 in self.object.shapes:
        buff.write(_struct_b.pack(val1.type))
        length = len(val1.dimensions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.dimensions.tostring())
        length = len(val1.triangles)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(val1.triangles.tostring())
        length = len(val1.vertices)
        buff.write(_struct_I.pack(length))
        for val2 in val1.vertices:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.object.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.object.poses:
        _v5 = val1.position
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = val1.orientation
        _x = _v6
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.touch_links)
      buff.write(_struct_I.pack(length))
      for val1 in self.touch_links:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.object is None:
        self.object = mapping_msgs.msg.CollisionObject()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.link_name = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.object.header.seq, _x.object.header.stamp.secs, _x.object.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.object.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.object.id = str[start:end]
      start = end
      end += 1
      (self.object.operation.operation,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object.shapes = []
      for i in range(0, length):
        val1 = geometric_shapes_msgs.msg.Shape()
        start = end
        end += 1
        (val1.type,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        val1.triangles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.vertices = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Point()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.vertices.append(val2)
        self.object.shapes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.object.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v7 = val1.position
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v8 = val1.orientation
        _x = _v8
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.object.poses.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.touch_links = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.touch_links.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_b = struct.Struct("<b")
_struct_4d = struct.Struct("<4d")
_struct_3d = struct.Struct("<3d")