#!/usr/bin/python

import sys
import os

fname = sys.argv[1]
inputfile = open(fname,'r')
lines_per_file = int(sys.argv[2])+1

directory = fname + '_split'
if not os.path.exists(directory):
  os.makedirs(directory)

line_count = 1
file_count = 1
outfile = open( directory+'/'+'%06d.txt'%(file_count), 'w' )
for line in inputfile:
  if ( line_count%lines_per_file == 0 ):
    outfile.close()
    file_count += 1	
    outfile = open( directory+'/'+'%06d.txt'%(file_count), 'w' )
    line_count = 1
  outfile.write(line)
  line_count += 1
  
if not outfile.closed:
  outfile.close()
