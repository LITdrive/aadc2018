# Team LITdrive - Audi Autonomous Driving Cup 2018

[![License](https://img.shields.io/badge/license-BSD%204--Clause-blue.svg)](LICENSE.txt)

This is the source code of **Team LITdrive**, the team of the [Johannes Kepler University Linz](https://www.jku.at/) for the [Audi Autonomous Driving Cup 2018](https://www.audi-autonomous-driving-cup.com/).
This repository serves as an archive for interested teams and as a starting point for future competitions.
You need the [ADAS car](https://www.bfft.de/wp-content/uploads/2017/06/produktreferenz_adas-modellfahrzeug.pdf) and the AADC code base in order to setup this project.

:car: :car: :car:

## Project Structure

- [`config`](/config) holds the ADTF projects *LiveVisualization* (for development) and *UserConfiguration* (competition graph)
- [`configuration_files`](/configuration_files) holds calibration files for the camera, machine learning models, properties, etc.
- [`data`](/data) holds map data for the localization filters
- [`description`](/description) holds the stream type definitions for ADTF
- [`doc/adtf-pitfalls`](/doc/adtf-pitfalls) holds some notes about common ADTF pitfalls
- [`include`](/include) holds headers and libraries (e.g., Eigen) which are used by all projects
- [`scripts`](/scripts) holds some scripts for running the ADTF sessions and preparing the environment
- [`src/aadcBase`](/src/aadcBase) holds filters provided by the organizers (arduino communication, jury interface, positioning, etc.)
- [`src/aadcDemo:`](/src/aadcDemo) holds demo filters for lane detection, object detection, controllers, sensor visualization and processing
- **[`src/aadcUser`](/src/aadcUser) holds the main filters for the competition, written in C++**
- **[`src/aadcUserPython`](/src/aadcUserPython)  holds the filters written in Python (and some additional notebooks)**

And some important files:

- The `build_*.sh` scripts build, compile and install the project
- [`AADCConfig.cmake`](AADCConfig.cmake) is the CMake configuration for configuring the entire project and all its dependencies

## Documentation

If you are working with ADTF, you might want to read about some [**Common ADTF Pitfalls**](/doc/adtf-pitfalls).

You might also want to use our famous ZeroMQ filter, which is used for pumping sensor data to a message queue and receiving it with another process, e.g. Python.
Find some details on [**how to use ZeroMQ filters here**](/doc/zeromq).

Additionally, you may find these external guides on ADTF very helpful:

- [**ADTF3 Guides:** Very beginner-friendly tutorial-style introduction (by Digitalwerk)](https://support.digitalwerk.net/adtf3_guides/index.html)
- [**ADTF3 Software Documentation:** The core objects have very useful documentation](https://support.digitalwerk.net/adtf/v3/adtf_html/index.html)

## Additional Libraries

The following additional libraries need to be installed on the car.

### ZeroMQ

[ZeroMQ](http://zeromq.org/) is a low-overhead, low-latency, high-speed IPC library, which is used for pumping sensor data to a message queue and receiving it with another program, e.g. Python.

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

### FFTW

[FFTW](http://www.fftw.org/) is a library for computing the discrete Fourier transform (DFT) in one or more dimensions, used on the microphone samples.

    cd /opt
    sudo wget http://www.fftw.org/fftw-3.3.8.tar.gz -O /tmp/fftw.tar.gz
    sudo mkdir fftw
    sudo tar -xzf /tmp/fftw.tar.gz -C fftw
    sudo chown -R aadc:aadc fftw
    cd /opt/fftw/fftw-3.3.8
    mkdir build; cd build
    cmake ..
    make
    echo '/opt/fftw/fftw-3.3.8/build/' | sudo tee /etc/ld.so.conf.d/fftw.conf
    sudo ldconfig

On Windows, download the 64 bit binaries from [here](ftp://ftp.fftw.org/pub/fftw/fftw-3.3.5-dll64.zip), extract the contents to `C:\SDK\fftw\fftw-3.3.5-dll64` and execute the following commands in a VS2015 Developer Console (to generate the `*.lib` files from the `*.dll` files):

    cd C:\SDK\fftw\fftw-3.3.5-dll64
    lib /machine:x64 /def:libfftw3f-3.def
    lib /machine:x64 /def:libfftw3-3.def
    lib /machine:x64 /def:libfftw3l-3.def

### NVIDIA Driver 396.xx

**Linux-only!** This NVIDIA driver enables you to run TensorFlow graphs on the GPU.

    sudo add-apt-repository ppa:graphics-drivers/ppa
    sudo apt update
    sudo apt install nvidia-396

Restart machine and check with

    nvidia-smi

### Eigen 3.3.4

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a header-only library for various linear algebra stuff (matrices, vectors, solvers, etc.).

**Linux-only!** Download the precompiled archive from [here](https://drive.google.com/file/d/1m8tXbVHjtSuV_cpZmR51T1Z4Kzz9et-3/view?usp=sharing), extract and copy this to `/opt/eigen/3.3.4` (such that you have `share` and `include` folders in there).

### Protobuf 3.5.0

[Protocol Buffers](https://developers.google.com/protocol-buffers/) are the binary message format for TensorFlow graph files.

**Linux-only!** We need to build Protobuf. Prepare the environment with

    sudo apt-get update
    sudo apt-get install autoconf automake libtool curl make g++ unzip

Next, download and extract the package

    cd /opt
    sudo wget https://github.com/protocolbuffers/protobuf/releases/download/v3.5.1/protobuf-cpp-3.5.1.tar.gz -O /tmp/protobuf.tar.gz
    sudo mkdir protobuf
    sudo tar -xzf /tmp/protobuf.tar.gz -C protobuf
    sudo chown -R aadc:aadc protobuf

Next, configure, make, install and update the shared library path

    cd /opt/protobuf/protobuf-3.5.1
    chmod +x configure
    ./configure
    make -j5
    make check -j5
    sudo make install
    sudo ldconfig

The libraries should have been installed to `/usr/local/lib`, you can check this with

    ls /usr/local/lib/ | grep proto

### TensorFlow 1.8.0

[TensorFlow](https://www.tensorflow.org/) is a machine learning framework.

**Linux-only!** Download the precompiled archive from [here](https://drive.google.com/file/d/1lY8VUlROLTkavQFePoHVidur-TpKr1fj/view?usp=sharing), extract and copy this to `/opt/tensorflow/1.8.0` (such that you have `external`, `include` and `lib` folders in there). Next, configure the shared library.

    echo '/opt/tensorflow/1.8.0/lib' | sudo tee /etc/ld.so.conf.d/tensorflow.conf
    sudo ldconfig

### Darknet

[Darknet](https://pjreddie.com/darknet/) is an open source neural network framework written in C and CUDA.

**Linux-only!** We need to build darknet from the sources. Darknet is built with CUDA support, but without OpenCV (the picture is already in raw format) or OpenMP.

        cd /opt
        sudo git clone https://github.com/pjreddie/darknet.git
        sudo chown aadc:aadc ./darknet -R
        cd darknet
        git apply /home/aadc/share/adtf/scripts/darknet_makefile.patch
        make -j4

Darknet is now built and ready!