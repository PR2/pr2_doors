from opencv.highgui import *
from opencv.cv import *
import numpy
import scipy
import sys
import os
from string import split

image = None
image_clone = None
in_draw = False

bb_list = []

def show_bb(image, bbs):
    pass


def on_mouse(event, x, y, flags, p ):
    global start, image, image_clone, in_draw

    if not image:
        return

    cvCopy(image_clone,image)

    if event == CV_EVENT_LBUTTONDOWN:
        start = (x,y)
        in_draw = True
    if event == CV_EVENT_MOUSEMOVE:
        if in_draw:
            start_x = min(start[0],x)
            start_y = min(start[1],y)
            end_x = max(start[0],x)
            end_y = max(start[1],y)
            cvRectangle(image, cvPoint(start_x,start_y), cvPoint(end_x,end_y),CV_RGB(255,0,0))
    elif event == CV_EVENT_LBUTTONUP:
        if in_draw:
            in_draw = False
            start_x = min(start[0],x)
            start_y = min(start[1],y)
            end_x = max(start[0],x)
            end_y = max(start[1],y)
            cvRectangle(image_clone, cvPoint(start_x,start_y), cvPoint(end_x,end_y),CV_RGB(255,0,0))
            cvRectangle(image, cvPoint(start_x,start_y), cvPoint(end_x,end_y),CV_RGB(255,0,0))
            bb_list.append((start_x,start_y,end_x,end_y))
        

    cvShowImage("image",image)


def read_bb(bb_name):
    global bb_dict
    bb_file = open(bb_name,"r")
    for line in bb_file:
        items = split(line)
        bb_dict[items[0]] = map(int,items[1:])
    bb_file.close()
    return bb_dict

def annotate(image_name, bb_name):
    global image, image_clone, bb_list
    #bb_dict = read_bb(bb_name)
    image = cvLoadImage(image_name)
    image_clone = cvCloneImage(image)


    cvNamedWindow("image",1)
    cvShowImage("image",image)
    cvSetMouseCallback( "image", on_mouse, None );

    while True:
        c = '%c' % (cvWaitKey(0) & 255)

        if( c == '\x1b' or c=='q'):
            print "Exiting, not saving to file"
            break;

        if c=='s':
            print "Exiting, saving to file: %s"%bb_name
            file = open(bb_name,"a")
            print >>file, image_name,
            for bb in bb_list:
                for x in bb:
                    print >>file, x,
            print >>file
            file.close()
            break;
        


if __name__=="__main__":
    if len(sys.argv)<3:
        print "Usage: annotate.py bb_file image"
        sys.exit(1)

    annotate(sys.argv[2],  sys.argv[1])



