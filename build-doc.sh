#!/bin/sh

set -e

BUILD_DIR=build_doc

rm -Rf $BUILD_DIR
mkdir $BUILD_DIR

cp -R doc/* $BUILD_DIR

cd $BUILD_DIR



lyx --export pdf physicsengine.lyx

echo done building doc
