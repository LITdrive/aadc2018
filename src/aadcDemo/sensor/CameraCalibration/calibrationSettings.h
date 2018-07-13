/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#include "stdafx.h"

/*! current state of filter */
enum { DETECTION = 0, CAPTURING = 1, CAPTURING_FINISHED = 2, WAITING = 3 };

/*! this structs holds all the settings for the calibration */
struct calibrationSettings
{
    /*! pattern which is used by calibration method */
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    /*! holds the number of squares given in the filter properties -> Number of items by width and height */
    Size boardSize;
    /*! the pattern which is used for calibration */
    Pattern calibrationPattern;
    /*! The size of a square in your defined unit (point, millimeter,etc) */
    float squareSize;
    /*! the minimum number of frames which are used for actual calibration */
    tUInt64 nrFrames;
	/*! the minimum number of frames which are used for inital calibration */
    tUInt64 nrFramesDataAq;
    /*! The aspect ratio */
    float aspectRatio;
    /*! the delay which must be between frames used for calibration (to secure that there is a difference in the images)*/
    tInt32 delay;
    /*! Write detected feature points */
    bool writePoints;
    /*! Write extrinsic parameters */
    bool writeExtrinsics;
    /*! The name of the file where to write */
    string outputFileName;
    /*! use fisheye camera model for calibration */
    bool useFisheye;
    /*! the flag for the calibration */
    int flag;
    /*! Assume zero tangential distortion */
    bool calibZeroTangentDist;
    /*! Fix the principal point at the center */
    bool calibFixPrincipalPoint;

};