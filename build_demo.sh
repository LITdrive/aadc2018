#!/bin/sh

# Parameter für Qt-Pfad

QT_DIR=/opt/qt/5.9.0/5.9/gcc_64/
ADTF_DIR=/opt/ADTF/3.3.1/

## if commandline --qt-dir
##  QT_DIR=from commandline
## fi

## if commandline --adtf-dir
##  ADTF_DIR=from commandline
## fi

QMAKE_PATH=$QT_DIR/bin/qmake

if which cmake > /dev/null; then
    echo "cmake found"
else
    echo "cmake not found. Make sure it's in your PATH."
    exit 1
fi

if [ -f $QMAKE_PATH ]; then
    echo "qmake found."
else
    echo "qmake not found in $QMAKE_PATH. Check the path to Qt or set it as commandline parameter."
    exit 1

fi

if [ -d $ADTF_DIR ]; then
    echo "ADTF dir found."
else
    echo "ADTF dir not found in $ADTF_DIR. Check the path to ADTF or set it as commandline parameter."
    exit 1
fi

## if commandline cleanup before build
##     rm -rf _build_demo
## fi

if [ -d ./_build_demo ]; then
    echo "Build exists. Will using it."
else
    mkdir ./_build_demo
    echo "Creating build directory."
fi

echo "entering build directory"
cd ./_build_demo

echo "generate cmake config"
cmake -G "Unix Makefiles" -DADTF_DIR=$ADTF_DIR -DQT_DIR=$QT_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo ../src/aadcDemo

echo "build ..."
cmake --build . --target install --config Debug -- -j4

cd ..





