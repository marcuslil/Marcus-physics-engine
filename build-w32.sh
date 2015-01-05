#!/bin/sh

set -e

BUILD_DIR=build_w32

rm -Rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

mkdir src
cd src
ln -sf ../../src/* .
cd ..

grep armadillo -v src/physicsengine.pro >src/physicsengine_for_win.pro
echo >>src/physicsengine_for_win.pro "QMAKE_LIBDIR  += ../lapack_3.5.0_win32"
echo >>src/physicsengine_for_win.pro "INCLUDEPATH  += ../lapack_3.5.0_win32"


cat >build.BAT << EOF
call C:\Qt\4.8.6\bin\qtvars
qmake src\physicsengine_for_win.pro
call make release -j 4
REM release\fysik1
REM cmd
EOF

WINEPREFIX=/sharedwine/default wineconsole build.BAT

echo done building w32
