#!/bin/bash

MAKEFILE=Makefile.gpp

EXAMPLES=(balljoints breakageobjects car hingejointmotoron hingejoints raddude rigidparticlesandterrain sccollision stackingobjects tank)
LIBSOURCES=(boxcylinder collisionbody collision constraint cylinder dcd lines ne_interface perflinux region restcontact rigidbodybase rigidbody scenery simulator solver sphere stack tricollision)
SAMPLEAPP=(tok_sample_glapp)
SAMPLESOURCES=("${EXAMPLES[@]}" "${SAMPLEAPP[@]}")

echo "CXXFLAGS=-Wno-unknown-pragmas -O2 -flto -fno-fat-lto-objects -Wall -Wextra -fPIC" > $MAKEFILE
echo "GPP=g++" >> $MAKEFILE
echo >> $MAKEFILE
echo -n "all:" >> $MAKEFILE

for EXAMPLE in "${EXAMPLES[@]}"
do
    echo -n " $EXAMPLE" >> $MAKEFILE
done

echo >> $MAKEFILE
echo >> $MAKEFILE
echo "clean:" >> $MAKEFILE
echo -n -e "\trm libtokamak.a *.o" >> $MAKEFILE
for EXAMPLE in "${EXAMPLES[@]}"
do
    echo -n " $EXAMPLE" >> $MAKEFILE
done
echo >> $MAKEFILE
echo >> $MAKEFILE

echo -n "libtokamak.a:" >> $MAKEFILE
for LIBSOURCE in "${LIBSOURCES[@]}"
do
    echo -n " $LIBSOURCE.o" >> $MAKEFILE
done
echo >> $MAKEFILE
echo -n -e "\tar rcs libtokamak.a" >> $MAKEFILE
for LIBSOURCE in "${LIBSOURCES[@]}"
do
    echo -n " $LIBSOURCE.o" >> $MAKEFILE
done
echo >> $MAKEFILE
echo -e "\tranlib libtokamak.a" >> $MAKEFILE

echo >> $MAKEFILE

for LIBSOURCE in "${LIBSOURCES[@]}"
do
    echo >> $MAKEFILE    
    echo "$LIBSOURCE.o: ../tokamaksrc/src/$LIBSOURCE.cpp" >> $MAKEFILE
    echo -n -e "\t$" >> $MAKEFILE
    echo -n "(GPP) $" >> $MAKEFILE
    echo "(CXXFLAGS) -c -I../include ../tokamaksrc/src/$LIBSOURCE.cpp -o $LIBSOURCE.o" >> $MAKEFILE
done

for SOURCE in "${SAMPLESOURCES[@]}"
do
    echo >> $MAKEFILE    
    echo "$SOURCE.o: $SOURCE.cpp" >> $MAKEFILE
    echo -n -e "\t$" >> $MAKEFILE
    echo -n "(GPP) $" >> $MAKEFILE
    echo "(CXXFLAGS) -c -I../include $SOURCE.cpp -o $SOURCE.o" >> $MAKEFILE
done

for EXAMPLE in "${EXAMPLES[@]}"
do
    echo >> $MAKEFILE    
    echo "$EXAMPLE: $EXAMPLE.o $SAMPLEAPP.o libtokamak.a" >> $MAKEFILE
    echo -n -e "\t$" >> $MAKEFILE
    echo "(GPP)  -Wl,-O1 -pipe -O2 -flto=6 -fno-fat-lto-objects -fuse-linker-plugin -fPIC -o $EXAMPLE $EXAMPLE.o $SAMPLEAPP.o -L./ -lSDL2 -lGL -ltokamak " >> $MAKEFILE
done
