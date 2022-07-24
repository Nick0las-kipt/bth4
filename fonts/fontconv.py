#!/usr/bin/python2
import sys
from PIL import Image

sym_count=11

if (len(sys.argv) < 2):
    print("usage %s <image>"%(sys.argv[0]))
    sys.exit(1)

im = Image.open(sys.argv[1])
width, height = im.size
if (0 != height % sym_count):
    print("There should be 11 equal symbols vertial")
    sys.exit(1)
sym_height = height / sym_count;
mask=[]

for row in range(height):
    val=0
    for col in range(width):
        if (im.getpixel((col,row)) > 0):
            val += 1
    mask.append(val)


stream=[]
pointers=[]

full_mask=[]
for row in range(sym_height):
    val = 0
#    vals=[]
    for asym in range(sym_count):
       val += mask[row + asym*sym_height]
#       vals.append(mask[row + asym*sym_height])
#    print("Line %d: %s"%(row,str(vals)))
    full_mask.append(val)

f_first = 0;
f_last = sym_height - 1
while ((f_first < sym_height) and (0 == full_mask[f_first])):
    f_first += 1

while ((f_first < f_last) and (0 == full_mask[f_last])):
    f_last -= 1

for symbol in range(sym_count):
    srow = symbol * sym_height
    pointers.append(len(stream))
    vrow = f_first
    for rrow in range(f_first, f_last+1):
        row=srow+rrow
        if (mask[row] > 0):
            if (rrow > vrow):
                stream.append(128+rrow-vrow)
            vrow=1+rrow
            val=0
            pos=0
            arr=[]
            for col in range(width):
                nval = 1 if (im.getpixel((col,row)) > 0) else 0
                if ((val != nval)):
                    arr.append(col-pos)
                    val=nval
                    pos=col
            if(0 == len(arr)):
                arr.append(0)
            arr[-1] += 64
            #print("Line %d: %s"%(row, str(arr)))
            stream.extend(arr)
#print(stream)
pointers.append(len(stream))
print("const int clock_sym_w = %d;"%(width))
print("const int clock_sym_h = %d;"%(f_last+1-f_first))
print("const int clock_sym_offsets[] = {%s};"%(str(pointers).strip('[]')))
print("const uint8_t clock_sym_data[] = {")
for line in range(sym_count):
    substream=stream[pointers[line]:pointers[1+line]]
    print("     %s,"%(str(substream).strip('[]')))
print("};")