from opencv import *
import numpy
import scipy
import sys
import os
from string import split, strip
import itertools
from array import array
from scipy.io.numpyio import fwrite, fread




def group(lst, n):
    """group([0,3,4,10,2,3], 2) => iterator

    Group an iterable into an n-tuples iterable. Incomplete tuples
    are discarded e.g.

    >>> list(group(range(10), 3))
    [(0, 1, 2), (3, 4, 5), (6, 7, 8)]
    """
    return itertools.izip(*[itertools.islice(lst, i, None, n) for i in range(n)])


def read_bb(bb_name):
    bb_dict = {}
    bb_file = open(bb_name,"r")
    for line in bb_file:
        items = split(line)
        bb_dict[items[0]] = map(int,items[1:])
    bb_file.close()
    return bb_dict


def convert(bb_name, output_file):
    bb_dict = read_bb(bb_name)
    bb_cv_dict = {}
    file = open(output_file,"w")
    for key, bb_list in bb_dict.items():
        count = 0
        for i in range(0,len(bb_list),4):
            bb_list[i+2] -= bb_list[i]
            bb_list[i+3] -= bb_list[i+1]
            count +=1
        print >>file, key, count,
        for bb in bb_list:
            print >>file, bb,
        print >>file
    
if __name__=="__main__":
    if len(sys.argv)<2:
        print "Usage: %s input_file output_file"%sys.argv[0]
        sys.exit(1)

    convert(sys.argv[1], sys.argv[2])
    