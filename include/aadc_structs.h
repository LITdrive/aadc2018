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

#ifndef AADC_STRUCTS_H
#define AADC_STRUCTS_H

#pragma pack(push,1)
typedef struct
{
    tInt16 i16ActionID;
    tInt16 i16ManeuverEntry;
} tJuryStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16StateID;
    tInt16 i16ManeuverEntry;
} tDriverStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32Value;
} tSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tUInt32 ui32WheelTach;
    tInt8 i8WheelDir;
} tWheelData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32A_x;
    tFloat32 f32A_y;
    tFloat32 f32A_z;
    tFloat32 f32G_x;
    tFloat32 f32G_y;
    tFloat32 f32G_z;
    tFloat32 f32M_x;
    tFloat32 f32M_y;
    tFloat32 f32M_z;
} tInerMeasUnitData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32Imagesize;
    tFloat32 af32TVec[3];
    tFloat32 af32RVec[3];
} tRoadSignExt;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
} tPosition;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
} tObstacle;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32angle;
} tTrafficSign;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32x;
    tFloat32 f32y;
    tUInt16 ui16Status;
} tParkingSpace;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSignalValue tSideLeft;
    tSignalValue tSideRight;
    tSignalValue tRearLeft;
    tSignalValue tRearCenter;
    tSignalValue tRearRight;
} tUltrasonicStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSignalValue tActuatorVoltage;
    tSignalValue tActuatorCell1;
    tSignalValue tActuatorCell2;
    tSignalValue tSensorVoltage;
    tSignalValue tSensorCell1;
    tSignalValue tSensorCell2;
    tSignalValue tSensorCell3;
    tSignalValue tSensorCell4;
    tSignalValue tSensorCell5;
    tSignalValue tSensorCell6;
} tVoltageStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32Radius;
    tFloat32 f32Angle;
} tPolarCoordiante;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32Size;
    tPolarCoordiante tScanArray[360];
} tLaserScannerData;
#pragma pack(pop)

// The following types are assumed to be known:
// tInt8
// tInt16
// tUInt32
// tFloat32
// tBool
// tUInt16

#endif