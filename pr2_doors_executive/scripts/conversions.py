#! /usr/bin/env python      

PKG = 'tf'
import roslib
roslib.load_manifest(PKG)

from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import PyKDL 
import tf
import rospy
import numpy


# to and from tf object
def fromTf(tf):
    position, quaternion = tf
    x, y, z = position
    Qx, Qy, Qz, Qw = quaternion
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(Qx, Qy, Qz, Qw), 
                       PyKDL.Vector(x, y, z))

def toTf(f):
    return ((f.p[0], f.p[1], f.p[2]),f.M.GetQuaternion())



# to and from pose message
def fromMsg(p):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(p.orientation.x,
                                                 p.orientation.y,
                                                 p.orientation.z,
                                                 p.orientation.w),  
                       PyKDL.Vector(p.position.x, p.position.y, p.position.z))

def toMsg(f):
    p = Pose()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = f.M.GetQuaternion()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    return p


# to and from matrix
def fromMatrix(m):
    return PyKDL.Frame(PyKDL.Rotation(m[0,0], m[0,1], m[0,2],
                                      m[1,0], m[1,1], m[1,2],
                                      m[2,0], m[2,1], m[2,2]),  
                       PyKDL.Vector(m[0,3], m[1, 3], m[2, 3]))

def toMatrix(f):
    return numpy.array([[f.M[0,0], f.M[0,1], f.M[0,2], f.p[0]],
                        [f.M[1,0], f.M[1,1], f.M[1,2], f.p[1]],
                        [f.M[2,0], f.M[2,1], f.M[2,2], f.p[2]],
                        [0,0,0,1]])


# from camera parameters
def fromCameraParams(cv, rvec, tvec):
    m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                      [ 0, 0, 0, tvec[1,0] ], 
                      [ 0, 0, 0, tvec[2,0] ], 
                      [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
    cv.Rodrigues2(rvec, m[:3,:3])
    return fromMatrix(m)




def main():
    print "\n\n==== To and from tf"
    tf = ((1,2,3), (1,0,0,0))
    print tf
    print toTf(fromTf(tf))
    
    print "\n\n==== To and from msg"
    p = Pose()
    p.orientation.x = 1
    p.position.x = 4
    print p
    print toMsg(fromMsg(p))

    print "\n\n==== To and from matrix"
    m = numpy.array([[0,1,0,2], [0,0,1,3],[1,0,0,4],[0,0,0,1]])
    print m
    print toMatrix(fromMatrix(m))

    print "\n\n==== Math"
    print toTf( fromMsg(p) * fromMatrix(m))
    print PyKDL.diff(fromMsg(p), fromMatrix(m))
    print PyKDL.addDelta(fromMsg(p), PyKDL.diff(fromMatrix(m), fromMsg(p)), 2.0)

if __name__ == '__main__': main()
