REM load the VS2015 environment
REM call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64

REM set the location of adtf
SET ADTF_DIR=C:\adtf\3.3.1

REM initialize cmake and create the build
cmake -G "Visual Studio 14 2015 Win64" -DCMAKE_C_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio 14.0/VC/bin/cl.exe" -DCMAKE_CXX_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio 14.0/VC/bin/cl.exe" .
