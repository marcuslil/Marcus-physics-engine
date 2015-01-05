#!/bin/sh

set -e

BUILD_DIR=build_linux

rm -Rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

qmake-qt4 ../src/physicsengine.pro
make -j 4

echo done building linux
