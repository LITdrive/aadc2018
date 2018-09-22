/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2018-05-15 10:15:43#$ $Rev:: 75429   $
**********************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// \file
///
/// \brief The arduino protocol definition.
///
/// This header define all structures which are used in the protocol. It defines
/// also some arduino specific stuff like arduino ids and samplingrates.
///
///////////////////////////////////////////////////////////////////////////////

#ifndef ARDUINO_PROTOCOL_HPP
#define ARDUINO_PROTOCOL_HPP

#include <stdint.h>

#pragma pack(push, 1)

///////////////////////////////////////////////////////////////////////////////
//
// Arduino frame definition
//
///////////////////////////////////////////////////////////////////////////////
/*

                +----+------------+------------+------+-------+
                | ID | Datalength | Timestamp  | Data | CRC16 |
                +----+------------+------------+------+-------+
Size (Byte):      1        1           4         var      2

+-------------------------------------------------------------+

Byte:           0    1            2            6      ?


*/

/*! A macro that defines start byte. */
#define START_BYTE      0x02    //STX

/*! A macro that defines end byte. */
#define END_BYTE        0x03    //ETX

/*! A macro that defines escape byte. */
#define ESCAPE_BYTE     0x1B    //ESCAPE

/*! A define for maximum trys to read a specific byte which is received by the arduino. */
#define MAX_MSG_LENGTH_TILL_TIMEOUT 64 // uint8_t

/*!  The maximum size of the std::vector implementation as implemented in utils_arduino.h */
#define MAX_VECTOR_BUFFER_SIZE 128

/*!
 * The Frame header definition of the protocol.
 * 
 */
struct tArduinoHeader
{
    /*! The header id */
    uint8_t   ui8ID;
    /*! Length of the data frame */
    uint8_t   ui8DataLength;
    /*! The arduino timestamp */
    uint32_t  ui32Timestamp;
};

/*!
 * All the sensor ids, which are set in tArduinoHeader.ui8ID.
 * 
 * To construct a frame with the specific id, create a tArduinoHeader struct and set the ui8ID
 * to that specific enum. This enums are used to send sensor information to a processing host.
 * The arduino system never processes these ids.
 */
enum SENSOR_ID /*: uint8_t*/{

    ID_ARD_SENSOR_INFO,
    ID_ARD_SENS_ERROR,

    ID_ARD_SENS_US_FRONT_LEFT,
    ID_ARD_SENS_US_FRONT_CENTER_LEFT,
    ID_ARD_SENS_US_FRONT_CENTER,
    ID_ARD_SENS_US_FRONT_CENTER_RIGHT,
    ID_ARD_SENS_US_FRONT_RIGHT,

    ID_ARD_SENS_US_REAR_RIGHT,
    ID_ARD_SENS_US_REAR_CENTER_RIGHT,
    ID_ARD_SENS_US_REAR_CENTER,
    ID_ARD_SENS_US_REAR_CENTER_LEFT,
    ID_ARD_SENS_US_REAR_LEFT,

    ID_ARD_SENS_US_SIDE_LEFT,
    ID_ARD_SENS_US_SIDE_RIGHT,

    ID_ARD_SENS_WHEEL_RIGHT,
    ID_ARD_SENS_WHEEL_LEFT,

    ID_ARD_SENS_IMU,

    ID_ARD_SENS_VOLT_ACTUATOR,
    ID_ARD_SENS_VOLT_ACTUATOR_CELL1,
    ID_ARD_SENS_VOLT_ACTUATOR_CELL2,

    ID_ARD_SENS_VOLT_SENSORS,
    ID_ARD_SENS_VOLT_SENSORS_CELL1,
    ID_ARD_SENS_VOLT_SENSORS_CELL2,
    ID_ARD_SENS_VOLT_SENSORS_CELL3,
    ID_ARD_SENS_VOLT_SENSORS_CELL4,
    ID_ARD_SENS_VOLT_SENSORS_CELL5,
    ID_ARD_SENS_VOLT_SENSORS_CELL6,
};

/*!
 * The Info struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position.
 */
struct tInfoData
{
    /*! see enum ARDUINO_ADDRESS for number(id) of the arduinos */
    uint8_t  ui8ArduinoAddress;
    /*! version of the arduino software */
    uint16_t ui16ArduinoVersion;
};

/*!
 * The Error struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The ui8ErrorNr is filled with an enum (ARD_ERROR) as described below.
 */
struct tErrorData
{
    /*! The error nr */
    int8_t ui8ErrorNr;
};

/*!
 * The Us data struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. Its member i16Distance is filled with the distance and its unit is [cm].
 */
struct tUsData
{
    /*! The distance in cm */
    int16_t i16Distance;
};

/*!
 * The wheel data struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. Its two members are used to indicate the tachometer and the direction of the driving
 * wheels. ui32WheelTach is the tick count of the wheels.
 */
struct tSensWheelData
{
    /*! The wheel tach */
    uint32_t ui32WheelTach;
    /*! The wheel direction */
    int8_t   i8WheelDir;
};

/*!
 * The imu data struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The units are ...acceleration [g], gyroscope [°/s] and for magnetometer [mG].
 */
struct tImuData
{
    /*! The ax */
    float f32ax;
    /*! The ay */
    float f32ay;
    /*! The az */
    float f32az;

    /*! The gx */
    float f32gx;
    /*! The gy */
    float f32gy;
    /*! The gz */
    float f32gz;

    /*! The mx */
    float f32mx;
    /*! The my */
    float f32my;
    /*! The mz */
    float f32mz;

    /*! The roll */
    float f32roll;
    /*! The pitch */
    float f32pitch;
    /*! The yaw */
    float f32yaw;
};

/*!
 * The voltage data struct.
 * 
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The unit is in milliVolt [mV].
 */
struct tVoltageData
{
    /*! Information describing the 16 voltage */
    uint16_t ui16VoltageData;
};

/*!
 * The data union for the protocol frame.
 * 
 * This union is a helper define for easier parsing by the receiver. The received frame could be
 * just copied into this union for easy access.
 */
union tDataUnion
{
    /*! The information */
    tInfoData info;
    /*! The error */
    tErrorData error;
    /*! The us distance */
    tUsData us;
    /*! The imu data*/
    tImuData imu;
    /*! The wheel data */
    tSensWheelData wheel;
    /*! The voltage data*/
    tVoltageData voltage;
};

/*!
 * The error codes as used by the ID_ARD_SENS_ERROR id.
 * 
 * The elements of this enums are used to fill the member of the tErrorData. The member
 * ui8ErrorNr should be set to one of this enum.
 */
enum ARD_ERROR_CODE /*: int8_t*/
{
    ERROR_STX_NOT_FOUND             = -1,
    ERROR_ETX_NOT_FOUND             = -2,
    ERROR_ESC_BYTE_BROKEN           = -3,
    ERROR_NO_BYTES_AVAILABLE        = -4,
    ERROR_CRC_INVALID               = -5,
    ERROR_NO_FRAME_DATA             = -6,
    ERROR_FRAME_DROPPED             = -7,
    ERROR_REMOTE_DETECTED           = -8,
    ERROR_NO_GYRO_DETECTED          = -9,
    ERROR_INVALID_ACTUATOR_HEADER   = -10,
    ERROR_INITIALIZATION_FAILED     = -11,
    ERROR_FRAME_NOT_WRITTEN         = -12,
};

/*!
 * The actuator ids. These ids are only processed by ARDUINO_CENTER_ACTUATORS.
 * 
 * The actuator ids are used to control the behavior of the arduino. For example send frames
 * from host with ID_ARD_ACT_STEER_SERVO or ID_ARD_ACT_SPEED_CONTR to control the steering and
 * throttle of the model car. Currently only the arduino with the id ARDUINO_CENTER_ACTUATORS
 * processes more than a ID_ARD_ACT_REQUEST frame. If a ID_ARD_ACT_REQUEST is sent to an arduino,
 * the arduino answer this request with an tInfoData.
 */
enum ACTUATOR_ID /*: uint8_t*/
{
    ID_ARD_ACT_NO_NAME          = 0,
    ID_ARD_ACT_WATCHDOG         = 1,
    ID_ARD_ACT_EMERGENCY_STOP   = 2,
    ID_ARD_ACT_REQUEST          = 3,

    ID_ARD_ACT_STEER_SERVO      = 4,
    ID_ARD_ACT_SPEED_CONTR      = 5,

    ID_ARD_ACT_LIGHT            = 6,

    ID_ARD_DISABLE_USS          = 7,
    ID_ARD_ENABLE_USS           = 8,
};

/*!
 * The Watchdog data. Processed by the ARDUINO_CENTER_ACTUATORS.
 * 
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. It should be received
 * every half a second. Otherwise the Servos stop working.
 */
struct tWatchdogData
{
    /*! The triggered state 1 = true,  0 = false */
    uint8_t ui8IsTriggerd;
} ;

/*!
 * The emergeny data. Processed by the ARDUINO_CENTER_ACTUATORS.
 * 
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. If this data is
 * received. The arduino shuts down the servos.
 */
struct tEmergencyStopData
{
    /*! The triggered state 1 = true,  0 = false */
    uint8_t ui8IsTriggerd;
};

/*!
 * The servo data. Processed by the ARDUINO_CENTER_ACTUATORS.
 * 
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. If this data is
 * received. The arduino sets the servo position (used by steering and throttle) accordingly.
 * The range is 0...180 with 90 as zero position.
 */
struct tServoData
{
    /*! The angle. Actually a servo number range is [0, 180]*/
    uint8_t ui8Angle;
};

/*!
 * The light data. Processed by the ARDUINO_CENTER_ACTUATORS.
 * 
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. This data is filled
 * with a member of the LIGHT_MASK enum. To set more lights than one the enums are combined with
 * each other.
 */
struct tLightData
{
    /*! see enum LIGHT_MASK for IDs */
    uint8_t ui8LightMask;
};

/*!
 * The light mask enum. Processed by the ARDUINO_CENTER_ACTUATORS.
 * 
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. The members of that
 * enum are used to set the ui8LightMask in tLightData.
 */
enum LIGHT_MASK /*: uint8_t*/
{
    ID_ARD_ACT_LIGHT_MASK_HEAD             = 0x01,
    //const uint8_t ID_ARD_ACT_LIGHT_MASK_BACK = 0x02; //same Pin as ID_ARD_ACT_LIGHT_MASK_HEAD
    ID_ARD_ACT_LIGHT_MASK_BRAKE            = 0x04,
    ID_ARD_ACT_LIGHT_MASK_TURNLEFT         = 0x08,
    ID_ARD_ACT_LIGHT_MASK_TURNRIGHT        = 0x10,
    ID_ARD_ACT_LIGHT_MASK_HAZARD           = ID_ARD_ACT_LIGHT_MASK_TURNLEFT | ID_ARD_ACT_LIGHT_MASK_TURNRIGHT,
    ID_ARD_ACT_LIGHT_MASK_REVERSE          = 0x20,
};

#pragma pack(pop)

///////////////////////////////////////////////////////////////////////////////
//
// Arduino pin defines and indentifiers
//
///////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/*
 |-------------|----------------|----------------------------------|
 | Arduino Nr. |  voltage [mV]  |            description           |
 |-------------|----------------|----------------------------------|
 |     1       |  155 < U < 175 |       Front with US sensors      |
 |     2       |  ??? < U < ??? |       Front with Gyro            |
 |     3       |  500 < U < 520 |       Center for measurements    |
 |     4       |  660 < U < 680 |       Center for actuators       |
 |     5       |  760 < U < 780 |       Rear US sensors            |
 |     6       |  920 < U < 940 |       Rear Gyro/wheelencoder     |
 |-------------|----------------|----------------------------------|
*/

/*!
 * The arduino identifiers.
 * 
 * These arduino ids are used to identify which arduino is communicating. When sending an info
 * frame to an arduino, it answers with a tInfoData with an memeber of the enum as id.
 */
enum ARDUINO_ID /*: uint8_t*/
{
	ARDUINO_NO_NAME                 = 0,
	ARDUINO_FRONT_US                = 1,
	ARDUINO_FRONT_IMU               = 2,
	ARDUINO_CENTER_MEASUREMENT      = 3,
	ARDUINO_CENTER_ACTUATORS        = 4,
	ARDUINO_REAR_US                 = 5,
	ARDUINO_REAR_IMU_WHEELENC       = 6,
	ARDUINO_TOP_RIGHT_OF_WAY		= 7,
};

 /*! A macro that defines pin ard code. */
#define PIN_ARD_CODE A3
/*! A macro that defines arduino 1 Minimum voltage. */
#define ARDUINO_1_MIN_VOLTAGE 155
/*! A macro that defines arduino 1 Maximum voltage. */
#define ARDUINO_1_MAX_VOLTAGE 175
/*! A macro that defines arduino 2 Minimum voltage. */
#define ARDUINO_2_MIN_VOLTAGE 0
/*! A macro that defines arduino 2 Maximum voltage. */
#define ARDUINO_2_MAX_VOLTAGE 0
/*! A macro that defines arduino 3 Minimum voltage. */
#define ARDUINO_3_MIN_VOLTAGE 500
/*! A macro that defines arduino 3 Maximum voltage. */
#define ARDUINO_3_MAX_VOLTAGE 520
/*! A macro that defines arduino 4 Minimum voltage. */
#define ARDUINO_4_MIN_VOLTAGE 660
/*! A macro that defines arduino 4 Maximum voltage. */
#define ARDUINO_4_MAX_VOLTAGE 680
/*! A macro that defines arduino 5 Minimum voltage. */
#define ARDUINO_5_MIN_VOLTAGE 760
/*! A macro that defines arduino 5 Maximum voltage. */
#define ARDUINO_5_MAX_VOLTAGE 780
/*! A macro that defines arduino 6 Minimum voltage. */
#define ARDUINO_6_MIN_VOLTAGE 920
/*! A macro that defines arduino 6 Maximum voltage. */
#define ARDUINO_6_MAX_VOLTAGE 1024

/*! A macro that defines Maximum range uss. */
#define MAX_RANGE_USS 400

/*! A macro that defines pin ana LED 0. */
#define PIN_ANA_LED0 int(13)
/*! A macro that defines pin ana LED 1. */
#define PIN_ANA_LED1 A0
/*! A macro that defines pin ana LED 2. */
#define PIN_ANA_LED2 A1

/*
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_CENTER (Nr. 1)
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 
 /*! A macro that defines pin us front left trigger. */
#define PIN_US_FRONT_LEFT_TRIGGER           int(9)

/*! A macro that defines pin us front left echo. */
#define PIN_US_FRONT_LEFT_ECHO              int(7)
/*! A macro that defines pin int us front left echo. */
#define PIN_INT_US_FRONT_LEFT_ECHO          int(4)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_FRONT_LEFT

/*! A macro that defines pin us front center left trigger. */
#define PIN_US_FRONT_CENTER_LEFT_TRIGGER    int(8)
/*! A macro that defines pin us front center left echo. */
#define PIN_US_FRONT_CENTER_LEFT_ECHO       int(3)
/*! A macro that defines pin int us front center left echo. */
#define PIN_INT_US_FRONT_CENTER_LEFT_ECHO   int(0)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_FRONT_CENTER_LEFT

/*! A macro that defines pin us front center trigger. */
#define PIN_US_FRONT_CENTER_TRIGGER         int(6)
/*! A macro that defines pin us front center echo. */
#define PIN_US_FRONT_CENTER_ECHO            int(2)
/*! A macro that defines pin int us front center echo. */
#define PIN_INT_US_FRONT_CENTER_ECHO        int(1)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_FRONT_CENTER

/*! A macro that defines pin us front center right trigger. */
#define PIN_US_FRONT_CENTER_RIGHT_TRIGGER   int(5)
/*! A macro that defines pin us front center right echo. */
#define PIN_US_FRONT_CENTER_RIGHT_ECHO      int(0)
/*! A macro that defines pin int us front center right echo. */
#define PIN_INT_US_FRONT_CENTER_RIGHT_ECHO  int(2)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_FRONT_CENTER_RIGHT

/*! A macro that defines pin us front right trigger. */
#define PIN_US_FRONT_RIGHT_TRIGGER          int(4)
/*! A macro that defines pin us front right echo. */
#define PIN_US_FRONT_RIGHT_ECHO             int(1)
/*! A macro that defines pin int us front right echo. */
#define PIN_INT_US_FRONT_RIGHT_ECHO         int(3)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_FRONT_RIGHT

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_FRONT_IMU (Nr. 2) also same pins as at arduino Nr. 6
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! A macro that defines pin int gyro. */
#define PIN_INT_GYRO        int(9)
/*! A macro that defines pin gyro vss enable. */
#define PIN_GYRO_VSS_ENABLE int(10)

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_CENTER_MEASUREMENT (Nr. 3)
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! A macro that defines pin ana volt actuator. */
#define PIN_ANA_VOLT_ACTUATOR       A6
/*! A macro that defines pin ana volt sensors. */
#define PIN_ANA_VOLT_SENSORS        A2

/*! A macro that defines pin ana volt actuator cell 1. */
#define PIN_ANA_VOLT_ACTUATOR_CELL1 A7  // Actuator Cell1
/*! A macro that defines pin ana volt actuator cell 2. */
#define PIN_ANA_VOLT_ACTUATOR_CELL2     // Actuator Cell2 = Actuator - BatteryCell1

/*! A macro that defines pin ana volt sensors cell 1. */
#define PIN_ANA_VOLT_SENSORS_CELL1  A5
/*! A macro that defines pin ana volt sensors cell 2. */
#define PIN_ANA_VOLT_SENSORS_CELL2  A4
/*! A macro that defines pin ana volt sensors cell 3. */
#define PIN_ANA_VOLT_SENSORS_CELL3  A8
/*! A macro that defines pin ana volt sensors cell 4. */
#define PIN_ANA_VOLT_SENSORS_CELL4  A9
/*! A macro that defines pin ana volt sensors cell 5. */
#define PIN_ANA_VOLT_SENSORS_CELL5  A10
/*! A macro that defines pin ana volt sensors cell 6. */
#define PIN_ANA_VOLT_SENSORS_CELL6  A11

/*! A macro that defines pin pipe sensors. */
#define PIN_PIPE_SENSORS            int(5)
/*! A macro that defines pin pipe actuator. */
#define PIN_PIPE_ACTUATOR           int(3)

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_CENTER_ACTUATORS (Nr. 4)
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! A macro that defines pin relais. */
#define PIN_RELAIS              	int(4)

/*! A macro that defines pin speed contr. */
#define PIN_SPEED_CONTR         	int(5)
/*! A macro that defines pin steer serv. */
#define PIN_STEER_SERV          	int(6)

/*! A macro that defines pin turnsignal left. */
#define PIN_TURNSIGNAL_LEFT     	int(11)
/*! A macro that defines pin turnsignal right. */
#define PIN_TURNSIGNAL_RIGHT    	int(10)
/*! A macro that defines pin brake light. */
#define PIN_BRAKE_LIGHT         	int(8)
/*! A macro that defines pin reverse light. */
#define PIN_REVERSE_LIGHT       	int(3)
/*! A macro that defines pin dim light. */
#define PIN_DIM_LIGHT           	int(12)

/*! A macro that defines pin ana poti steering. */
#define PIN_ANA_POTI_STEERING 	    A5
/*! A macro that defines pin ana poti speed. */
#define PIN_ANA_POTI_SPEED 	        A4

/*! A macro that defines pin remote receiver enabled. */
#define PIN_REMOTE_RECEIVER_ENABLED A2

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_REAR_US (Nr. 5)
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! A macro that defines pin us rear left trigger. */
#define PIN_US_REAR_LEFT_TRIGGER           int(4)
/*! A macro that defines pin us rear left echo. */
#define PIN_US_REAR_LEFT_ECHO              int(1)
/*! A macro that defines pin int us rear left echo. */
#define PIN_INT_US_REAR_LEFT_ECHO          int(3)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_REAR_LEFT

/*! A macro that defines pin us rear center left trigger. */
#define PIN_US_REAR_CENTER_LEFT_TRIGGER    int(5)
/*! A macro that defines pin us rear center left echo. */
#define PIN_US_REAR_CENTER_LEFT_ECHO       int(0)
/*! A macro that defines pin int us rear center left echo. */
#define PIN_INT_US_REAR_CENTER_LEFT_ECHO   int(2)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_REAR_CENTER_LEFT

/*! A macro that defines pin us rear center trigger. */
#define PIN_US_REAR_CENTER_TRIGGER         int(6)
/*! A macro that defines pin us rear center echo. */
#define PIN_US_REAR_CENTER_ECHO            int(2)
/*! A macro that defines pin int us rear center echo. */
#define PIN_INT_US_REAR_CENTER_ECHO        int(1)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_REAR_CENTER

/*! A macro that defines pin us rear center right trigger. */
#define PIN_US_REAR_CENTER_RIGHT_TRIGGER   int(8)
/*! A macro that defines pin us rear center right echo. */
#define PIN_US_REAR_CENTER_RIGHT_ECHO      int(3)
/*! A macro that defines pin int us rear center right echo. */
#define PIN_INT_US_REAR_CENTER_RIGHT_ECHO  int(0)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_REAR_CENTER_RIGHT

/*! A macro that defines pin us rear right trigger. */
#define PIN_US_REAR_RIGHT_TRIGGER          int(9)
/*! A macro that defines pin us rear right echo. */
#define PIN_US_REAR_RIGHT_ECHO             int(7)
/*! A macro that defines pin int us rear right echo. */
#define PIN_INT_US_REAR_RIGHT_ECHO         int(4)
/*! placeholder for VCC of pin */
#define PIN_VCC_US_REAR_RIGHT

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  PIN DEFINES FOR ARDUINO_REAR_IMU_WHEELENC (Nr. 6)
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! A macro that defines pin wheel left tach. */
#define PIN_WHEEL_LEFT_TACH         int(0)
/*! A macro that defines pin int wheel left tach. */
#define PIN_INT_WHEEL_LEFT_TACH     int(2)
/*! A macro that defines pin wheel left dir. */
#define PIN_WHEEL_LEFT_DIR          int(5)

/*! A macro that defines pin wheel right tach. */
#define PIN_WHEEL_RIGHT_TACH        int(1)
/*! A macro that defines pin int wheel right tach. */
#define PIN_INT_WHEEL_RIGHT_TACH    int(3)
/*! A macro that defines pin wheel right dir. */
#define PIN_WHEEL_RIGHT_DIR         int(4)

/*!
 * /////////////////////////////////////////////////////////////////////////////
 * 
 *  Sampling rate of all sensors in [Hz]
 * 
 * /////////////////////////////////////////////////////////////////////////////.
 */
 /*! The led0 sampling rate */
const unsigned long led0SamplingRate      = 2;
/*! The steering sampling rate */
const unsigned long SteeringSamplingRate  = 40;
/*! The wheel encode sampling rate */
const unsigned long WheelEncSamplingRate  = 40;
/*! The voltage sampling rate */
const unsigned long VoltageSamplingRate   = 2;
/*! The uss sampling rate */
const unsigned long UssSamplingRate       = 30;
/*! The watch dog sampling rate */
const unsigned long WatchDogSamplingRate  = 2;
/*! The light sampling rate */
const unsigned long LightSamplingRate     = 2;
/*! The imu sampling rate */
const unsigned long ImuSamplingRate       = 40;

#endif
