
import roslib; roslib.load_manifest('pr2_doors_executive')
import rospy

import math

from geometry_msgs.msg import *
from door_msgs.msg import *

import tf
import tf.transformations as transformations
import conversions

import PyKDL as kdl

eps_angle = 5.0*math.pi/180.0
gripper_height = 0.8

def point2kdl(point):
  return kdl.Vector(point.x, point.y, point.z)

def kdl2point(vector):
  return Point(vector.x, vector.y, vector.z)

def get_door_angle(door):
  frame_vec = kdl.Vector(
    door.frame_p1.x-door.frame_p2.x,
    door.frame_p1.y-door.frame_p2.y,
    door.frame_p1.z-door.frame_p2.z)
  door_vec = kdl.Vector(
    door.door_p1.x-door.door_p2.x,
    door.door_p1.y-door.door_p2.y,
    door.door_p1.z-door.door_p2.z)
  
  angle = get_yaw_angle(frame_vec, door_vec)

  if door.rot_dir == Door.ROT_DIR_CLOCKWISE and angle > eps_angle:
    rospy.logdebug("Door angle is positive, but door message specifies it turns clockwise")

  if door.rot_dir == Door.ROT_DIR_COUNTERCLOCKWISE and angle < -eps_angle:
    rospy.logdebug("Door angle is negative, but door message specifies it turns counter-clockwise")

  return angle

def get_door_normal(door):
  frame_normal = get_frame_normal(door)
  rot_frame_door = kdl.Rotation.RotZ(get_door_angle(door))
  return rot_frame_door * frame_normal

def get_frame_normal(door):
  """Get the normal of the door frame.
  @type door: door_msgs.msg.Door
  @rtype: kdl.Vector
  """
  # normal on frame
  p12 = kdl.Vector(
      door.frame_p1.x - door.frame_p2.x,
      door.frame_p1.y - door.frame_p2.y,
      door.frame_p1.z - door.frame_p2.z)

  p12.Normalize()
  z_axis = kdl.Vector(0,0,1)
  normal = p12 * z_axis

  # make normal point in direction we travel through door
  direction = kdl.Vector(
      door.travel_dir.x,
      door.travel_dir.y,
      door.travel_dir.z)

  if kdl.dot(direction, normal) < 0:
    normal = normal * -1.0

  return normal

def get_Frame_angle(door):
  if door.hinge == door.HINGE_P1:
    return math.atan2(door.frame_p2.y - door.frame_p1.y, door.frame_p2.x - door.frame_p1.x)
  else:
    return math.atan2(door.frame_p1.y - door.frame_p2.y, door.frame_p1.x - door.frame_p2.x)

def get_door_dir(door):
  # get frame vector
  if door.hinge == Door.HINGE_P1:
    frame_vec = point2kdl(door.frame_p2) - point2kdl(door.frame_p1)
  elif door.hinge == Door.HINGE_P2:
    frame_vec = point2kdl(door.frame_p1) - point2kdl(door.frame_p2)
  else:
    rospy.logerr("Hinge side is not defined")

  # rotate frame vector
  if door.rot_dir == Door.ROT_DIR_CLOCKWISE:
    frame_vec = kdl.Rotation.RotZ(-math.pi/2) * frame_vec
  elif door.rot_dir == Door.ROT_DIR_COUNTERCLOCKWISE:
    frame_vec = kdl.Rotation.RotZ(math.pi/2) * frame_vec
  else:
    rospy.logerr("Rot dir is not defined")

  if kdl.dot(frame_vec, get_frame_normal(door)) < 0:
    return -1.0 
  else:
    return 1.0

def get_handle_dir(door):
  frame_normal = get_frame_normal(door)

  if door.hinge == Door.HINGE_P1:
    frame_vec = point2kdl(door.frame_p2) - point2kdl(door.frame_p1)
  elif door.hinge == Door.HINGE_P2:
    frame_vec = point2kdl(door.frame_p1) - point2kdl(door.frame_p2)
  else:
    rospy.logerr("Hinge side is not defined")

  rot_vec = frame_vec * frame_normal
  if rot_vec[2] > 0:
    return -1.0
  else:
    return 1.0

def get_yaw_angle(v1, v2):
  """Get the angle between the projections of v1 and v2 onto the XY plane.
  @type v1: kdl.Vector
  @type v2: kdl.Vector
  @rtype: double
  """
  vec1 = v1
  vec2 = v2
  vec1.Normalize()
  vec2.Normalize()
  dot      = vec2[0] * vec1[0] + vec2[1] * vec1[1]
  perp_dot = vec2[1] * vec1[0] - vec2[0] * vec1[1]
  return math.atan2(perp_dot, dot)

def get_robot_pose(door, dist):
  """Get a robot pose some distance from the door.
  @type door: door_msgs.msg.Door
  @rtype: geometry_msgs.msg.PoseStamped
  """
  x_axis = kdl.Vector(1,0,0)

  frame_1 = kdl.Vector(door.frame_p1.x, door.frame_p1.y, door.frame_p1.z)
  frame_2 = kdl.Vector(door.frame_p2.x, door.frame_p2.y, door.frame_p2.z)
  frame_center = (frame_1+frame_2)/2.0

  frame_normal = get_frame_normal(door)
  robot_pos = frame_center + (frame_normal * dist)

  robot_pose = kdl.Frame(
      kdl.Rotation.RPY(0,0,get_yaw_angle(x_axis, frame_normal)),
      robot_pos)

  robot_pose_msg = PoseStamped() 
  robot_pose_msg.header.frame_id = door.header.frame_id
  robot_pose_msg.header.stamp = door.header.stamp
  robot_pose_msg.pose = conversions.toMsg(robot_pose)
  
  return robot_pose_msg

def get_gripper_pose(door, angle, dist, side):
    x_axis = kdl.Vector(1,0,0)
    
    # get hinge point
    hinge = kdl.Vector()
    frame_vec = kdl.Vector()

    if door.hinge == Door.HINGE_P1:
      hinge = kdl.Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z)
      frame_vec = kdl.Vector(
          door.frame_p2.x - door.frame_p1.x,
          door.frame_p2.y - door.frame_p1.y,
          door.frame_p2.z - door.frame_p1.z)
    elif door.hinge == Door.HINGE_P2:
      hinge = kdl.Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z)
      frame_vec = kdl.Vector(
          door.frame_p1.x - door.frame_p2.x,
          door.frame_p1.y - door.frame_p2.y,
          door.frame_p1.z - door.frame_p2.z)

    # get gripper pos
    frame_vec.Normalize()
    frame_vec = frame_vec * dist
    rot_angle = kdl.Rotation.RotZ(angle)
    gripper_pos = hinge + (rot_angle * frame_vec)
    
    frame_normal = get_frame_normal(door)

    if side == DoorCmd.PULL:
      axis_sign = -1.0
    else:
      axis_sign = 1.0

    gripper_pose = kdl.Frame(
      kdl.Rotation.RPY(0,0,get_yaw_angle(axis_sign*x_axis, frame_normal)+angle),
      gripper_pos)

    gripper_pose_msg = PoseStamped()
    gripper_pose_msg.header.frame_id = door.header.frame_id
    gripper_pose_msg.header.stamp = door.header.stamp
    gripper_pose_msg.pose = gripper_pose

    return gripper_pose_msg

def get_handle_pose(door, side):
  x_axis = kdl.Vector(1,0,0)
  
  dist = math.sqrt(
      math.pow(door.frame_p1.x - door.handle.x,2)
      +math.pow(door.frame_p1.y - door.handle.y,2))

  if door.hinge == Door.HINGE_P2:
    dist = math.sqrt(math.pow(door.frame_p2.x - door.handle.x,2)+math.pow(door.frame_p2.y - door.handle.y,2))

  angle = get_door_angle(door)

  # get hinge point

  if door.hinge == Door.HINGE_P1:
    hinge = point2kdl(door.door_p1)
    frame_vec = point2kdl(door.frame_p2) - point2kdl(door.frame_p1)
  else:
    hinge = point2kdl(door.door_p2)
    frame_vec = point2kdl(door.frame_p1) - point2kdl(door.frame_p2)

  # get gripper pos
  frame_vec.Normalize()
  frame_vec = frame_vec * dist
  rot_angle = kdl.Rotation.RotZ(angle)
  handle_pos = hinge + (rot_angle * frame_vec)
  
  # Construct handle pose
  frame_normal = get_frame_normal(door)
  if side == -1:
    frame_normal = -frame_normal

  handle_pose = kdl.Frame(
      kdl.Rotation.RPY(0,0,get_yaw_angle(x_axis, frame_normal)),
      handle_pos)

  gripper_rotate = kdl.Frame(
      kdl.Rotation.RPY(math.pi/2,0.0,0.0),
      kdl.Vector(0,0,0))
  
  handle_pose = handle_pose * gripper_rotate

  handle_pose_msg = PoseStamped()
  handle_pose_msg.header.frame_id = door.header.frame_id
  handle_pose_msg.header.stamp = door.header.stamp
  handle_pose_msg.pose = conversions.toMsg(handle_pose)

  return handle_pose_msg
