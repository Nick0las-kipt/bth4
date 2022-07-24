#!/usr/bin/python2
import sys
from PIL import Image

sym_count=11

if (len(sys.argv) < 2):
    print("usage %s <image>"%(sys.argv[0]))
    sys.exit(1)

im = Image.open(sys.argv[1])
width, height = im.size

for sym in range(107):
    sym_x = (sym % 32) * 12
    sym_y = (sym / 32) * 16
    numbit = 0
    byte = 0
    print("{ "),
    for y in range(16):
        for x in range(12):
            bit=0
            # print(im.getpixel((sym_x + x,sym_y +y))[0])
            if (im.getpixel((sym_x + x,sym_y +y))[0] == 0):
                bit = 1
            byte += bit << numbit
            numbit += 1
            if (numbit > 7):
                numbit = 0
                print("0x%02x,"%(byte)),
                byte = 0;
    print(" },")
    