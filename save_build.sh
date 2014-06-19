#!/bin/sh

set -e


./build-w32.sh
./build-linux.sh

SAVE_DIR=saved_builds

LATEST=$(cat $SAVE_DIR/latest)
NEXT=$((LATEST+1))


TIME=`date +"%Y-%m-%d_%H_%M_%S"`
NAME=${TIME}_nr_${NEXT}
DIR="build_${NAME}"

mkdir -p $SAVE_DIR/$DIR
tar cj src >$SAVE_DIR/$DIR/src.tar.bz
cd $SAVE_DIR
cp ../build_w32/release/fysik1.exe  $DIR/fysik1_w32_${NAME}.exe
cp ../build_linux/fysik1 $DIR/fysik_$(arch)_${NAME}

zip $DIR.zip -r $DIR
mkdir -p ~/Dropbox/Public/fysikmotor
cp $DIR.zip ~/Dropbox/Public/fysikmotor

echo $NEXT > latest

