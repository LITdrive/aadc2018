#!/bin/sh

# return on errors
set -e

if which cmake > /dev/null; then
    echo "cmake found"
else
    echo "cmake not found. Make sure it's in your PATH."
    exit 1
fi


## if commandline cleanup before build
#     rm -rf _build_base
## fi

## configure debug

if [ -d ./_build_base_debug ]; then
    echo "Debug build exists. Will using it."
else
    mkdir ./_build_base_debug
    echo "Creating build directory _build_user_debug."
fi

cd ./_build_base_debug

echo "Generate cmake config debug"
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ../src/aadcBase

## build debug

echo "Build debug ..."
cmake --build . --target install -- -j4

## copy plugin description
find . -type f -name "*.plugindescription" -print0 | xargs -0 cp -t ../_install/linux64/bin/debug/

cd ..


## configure release
if [ -d ./_build_base ]; then
    echo "Build exists. Will using it."
else
    mkdir ./_build_base
    echo "Creating build directory _build_user"
fi

echo "Entering build directory"
cd ./_build_base

echo "generate cmake config"
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../src/aadcBase

## build release

echo "Build release ..."
cmake --build . --target install -- -j4

## copy plugin description
find . -type f -name "*.plugindescription" -print0 | xargs -0 cp -t ../_install/linux64/bin/

cd ..


