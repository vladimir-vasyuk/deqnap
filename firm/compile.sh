#!/bin/sh
./macro11 -o firmdeqna.obj -l firmdeqna.lst firmdeqna.mac
./rt11obj2bin firmdeqna.obj > firmdeqna.map
srec_cat firmdeqna.obj.bin -binary --byte-swap 2 -fill 0x00 0x0000 0x1000 -o firmdeqna.mif -Memory_Initialization_File 16 -obs=2

