#!/bin/sh
./convboot deqnaboot.bin
if [ $? -eq 0 ]; then
   srec_cat deqnaboot.bin -binary --byte-swap 2 -o deqnaboot.mif -Memory_Initialization_File 16 -obs=2
   rm deqnaboot.bin
fi
