#!/bin/sh

set -e

BUILD_DIR=build_w32

grep armadillo -v src/physicsengine.pro >src/physicsengine_for_win.pro

rm -Rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR


cat >build.BAT << EOF
call C:\Qt\4.8.5\bin\qtvars
qmake ..\src\physicsengine_for_win.pro
call make release
REM release\fysik1
REM cmd
EOF

WINEPREFIX=/sharedwine/default wineconsole build.BAT


echo done building w32
