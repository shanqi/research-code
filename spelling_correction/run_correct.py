#!/usr/bin/python

import sys
from correct_v1 import correct
from tokenize import generate_tokens

def is_number(s):
  try:
    float(s)
    return True
  except ValueError:
    return False

fname = sys.argv[1]
outfilename = fname + '.corrected.txt'
print fname + ' -> ' + outfilename
inputfile = open(fname,'r')
outputfile = open(outfilename,'w')
line_count = 0
for line in inputfile:
  g = line.split(',')
  outputfile.write(g[0]+',')
  gsplit = g[1].split(' ')
  for entry in gsplit:
    entryClean = entry.replace('\n','').lower()
    if (is_number(entryClean)):
      outputfile.write(entryClean+' ')
    else:
      correctedWord = correct(entryClean)
      outputfile.write(correctedWord+' ')
  outputfile.seek(-1,2)
  outputfile.write('\n')
  line_count += 1
  if ( line_count%100 == 0 ):
    print 'line_count: ' + str(line_count)

inputfile.close()
outputfile.close()


