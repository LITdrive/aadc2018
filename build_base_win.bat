@echo off

REM Check if cmake is avaliable
cmake --version > nul 2>&1
if errorlevel 1 echo Unable to find cmake! Please make sure that it is somewhere in your PATH. You can use the installer available in bin\extern of your ADTF installation. & pause & exit /b
echo CMake found
REM Set Compiler String
set VS_STRING=Visual Studio 14 2015 Win64


FOR /F "tokens=*" %%i in ('cd') do set SOURCE_DIR=%%i
echo Source directory is "%SOURCE_DIR%"

set SOURCE_DIR=%SOURCE_DIR:\=/%
set INSTALL_DIR=%SOURCE_DIR%

set BUILD_DIR=%1
if "%BUILD_DIR%"=="" set BUILD_DIR=_build_base
echo Creating build system in "%BUILD_DIR%"
mkdir "%BUILD_DIR%" > nul 2>&1
cd "%BUILD_DIR%"



cmake -G "%VS_STRING%" "%SOURCE_DIR%" -DCMAKE_BUILD_TYPE=RelWithDebInfo ../src/aadcBase
if errorlevel 1 pause & exit /b

echo.
echo Finished! Your Visual Studio Solution has been created in "%BUILD_DIR%"
pause

