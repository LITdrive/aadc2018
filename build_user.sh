#!/bin/sh


if which cmake > /dev/null; then
    echo "cmake found"
else
    echo "cmake not found. Make sure it's in your PATH."
    exit 1
fi

## if commandline cleanup before build
##     rm -rf _build_user
## fi

## configure debug

if [ -d ./_build_user_debug ]; then
    echo "Debug build exists. Will using it."
else
    mkdir ./_build_user_debug
    echo "Creating build directory _build_user_debug."
fi

cd ./_build_user_debug

echo "Generate cmake config debug"
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ../src/aadcUser

## build debug

echo "Build debug ..."
cmake --build . --target install -- -j4

cd ..


## configure release
if [ -d ./_build_user ]; then
    echo "Build exists. Will using it."
else
    mkdir ./_build_user
    echo "Creating build directory _build_user"
fi

echo "Entering build directory"
cd ./_build_user

echo "generate cmake config"
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../src/aadcUser

## build release

echo "Build release ..."
cmake --build . --target install -- -j4

cd ..
