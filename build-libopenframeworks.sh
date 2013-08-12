#!/bin/bash
# build script for libopenframeworks

IFS=';'
DIRLIST='example-encode;example-decode;example-calibrate;example-triangulate;example-pcl'

mkdir -p bin

for d in ${DIRLIST}
do
	cd ${d}
	mkdir -p build
	mkdir -p share
	cd build
	cmake -DCMAKE_PREFIX_PATH=~/opt/lib/cmake/of ..
	make
	cp -p example_project ../../bin/${d}
	cd ../..
done
