# AADC 2018 - LITdrive

This is the main folder for the ADTF plugin development and the running of ADTF projects and contains all the delivered sources.

## Additional libraries

The following additional libraries need to be installed on the car.

### ZeroMQ

ZeroMQ is a low-overhead, low-latency, high-speed IPC library, which is used for pumping sensor data to a message queue and receiving it with another program, e.g. Python.

    cd /opt
    sudo git clone https://github.com/zeromq/libzmq.git
    sudo chown -R aadc:aadc libzmq
    cd /opt/libzmq
    mkdir build; cd build
    cmake ..
    make
	
On Windows, do the following:

	cd C:\SDKs
	git clone https://github.com/zeromq/libzmq.git
	md build; cd build
	cmake -G "Visual Studio 14 2015 Win64" ..
	cmake --build . --target INSTALL --config Release

## Structure

### Folders

- `_build_user:` temporary folder which is generated by calling `build_user.sh` or `build_user_win.bat` and is used while building plugins
- `_build_base:` temporary folder which is generated by calling `build_base.sh` or `build_base_win.bat` and is used while building plugins
- `_build_demo:` temporary folder which is generated by calling `build_demo.sh` or `build_demo_win.bat` and is used while building plugins
- `_install:` temporary folder which is generated in build process and contains all the generated binaries
- `config:` Some demo ADTF configurations can be found here
- `configuration_files:` Here a lot of useful files located, e.g. calibration files for the camera, an example of a manueverlist and many more
- `doc:` Documentation folder
- `include:` Some general sources are located here which are used by all projects
- `src/aadcUser:` Here are some template plugins for the teams located. The teams are advised to include all their filters in this project.
- `src/aadcBase:` In the base folder are the ADTF base plugins contained which must be used by all teams. The final project running at the competition must contain these filters.
- `src/aadcDemo:` Here are a lot of demo filters  contained which can be used by the teams or can also be modified by the teams on their own.

### Files

- `build_user.sh:` Builds and compiles the sources contained in `src/aadcUser` for Linux
- `build_base.sh:` Builds and compiles the sources contained in `src/aadcBase` for Linux
- `build_demo.sh:` Builds and compiles the sources contained in `src/aadcDemo` for Linux
- `build_user_win.bat:` Builds and compiles the sources contained in `src/aadcUser` for windows
- `build_base_win.bat:` Builds and compiles the sources contained in `src/aadcBase` for windows
- `build_demo_win.bat:` Builds and compiles the sources contained in `src/aadcDemo` for windows
- `AADCConfig.cmake:` configuration file for CMake which configures are all the used SDKs
- `AADC_PRIVATE.cmake:` contains some macros for use in CMake
- `AADCConfigVersion.cmake:` holds the version numbers of the current source
- `changelog.txt:` overview of the changes in source
- `LICENSE.txt:` license information
- `README.txt:` this file
