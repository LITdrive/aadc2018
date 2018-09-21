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

#include "stdafx.h"
#include "LITD_StanleyControl.h"
#include "math_utilities.h"
#include <ADTF3_helper.h>

/* notes to check
[] actual speed has to be in car-struct
[] is the given point normal to car position?

*/


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LITD_STANLEY_CONTROL_FILTER,
    "LITD_StanleyControl",
    cStanleyControl,
    adtf::filter::pin_trigger({"inPositionIs"}));

void cStanleyControl::calculateFrontPos(){
    double dx = cos(carBackPosition.h)*CAR_AXIS_DIST;
    double dy = sin(carBackPosition.h)*CAR_AXIS_DIST;

    carFrontPosition.x = carBackPosition.x + dx;
    carFrontPosition.y = carBackPosition.y + dy;
    carFrontPosition.h = carBackPosition.h;

}

void cStanleyControl::mapSteeringAngle(){
    tFloat32  rad2degree  = 180.0 / M_PI;
    carSteeringValue = (carSteeringAngle * rad2degree) / maxAngle * (-100);   
    

}


void cStanleyControl::calcSteeringAngle(){
    
    //vector between car and virtualpoint
    Vector2d diff = vp.getVector2d() - carFrontPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    tFloat32 diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vp.h))> 4.712 ){
        sign = -1;
    }

    //calc normal distance of tangent to car (e)
    tFloat32 e = (vp.getVector2d() - carFrontPosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    tFloat32 theta_c =  wrapTo2Pi(vp.h) - wrapTo2Pi(carFrontPosition.h);

    //calc steering-angle with stanley-approach
    carSteeringAngle = theta_c + atan2(stanleyGain*e, carSpeed);
    LOG_INFO("Steering angle in rad : %.2f ", carSteeringAngle);

    //Debug Messages
    /*std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    std::cout << "Steering Angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;*/
	LOG_INFO("point heading in rad : %.2f ", vp.h);
    LOG_INFO("car heading in rad : %.2f ", carFrontPosition.h);
	LOG_INFO("e  : %.2f ", e);
	LOG_INFO("theta : %.2f ", theta_c);
    LOG_INFO("car speed : %.2f ", carSpeed);
    LOG_INFO("calc : %.2f ", theta_c + atan2(stanleyGain*e, carSpeed));
    LOG_INFO("gainz : %.2f ", stanleyGain);

}
cStanleyControl::cStanleyControl()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypePositionData;
	object_ptr<IStreamType> pTypeSignalValue;

    RegisterPropertyVariable("dynamic properties path", m_properties_file);

    // load properties file for dynamic properties
	m_properties = new FilePropertiesObserver(static_cast<string>(cString(m_properties_file)));
	m_properties->ReloadProperties();



    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_VirtualPointSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }


	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
	{
		adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
		adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
	}
	else
	{
		LOG_INFO("No mediadescription for tSignalValue found!");
	}

    //Register input pin
    Register(m_oVPReaderIst, "inPositionIs", pTypePositionData);
    Register(m_oWriter, "output", pTypeSignalValue);


    LOG_INFO("Generating lane map...");

   //Vertical straight between x 0->2
  /*
  if(map.addStraightElement(0.0, 2.0, -0.5, 0.5, 0.0, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 0/0->2/0");
  }
  */
  if(map.addDoubleStraightElement(1.0, 4.07, -10.0, 1.0, 0.3, 0.775, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 1/0->4/1");
  }
  /*
  if(map.addCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 2/0->3/1");
  }
  */
  if(map.addDoubleCurveElement(4.07, 10.0, -10.0, 1.0, CURVE_CORNER_UL, 0.225, 0.7)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 4/0->5/1");
  }
  /*
  if(map.addStraightElement(2.5, 3.5, 1.0, 3.0, 3.0, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 3/1->3/3");
  }
  */

  if(map.addDoubleStraightElement(4.07, 10.0, 1.0, 2.0, 4.3, 4.75, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 4/1->5/2");
  }
  /*
  if(map.addCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 3/3->2/4");
  }
  */
  if(map.addDoubleCurveElement(3.0, 10.0, 2.0, 10.0, CURVE_CORNER_LL, 1.3, 1.75)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 3/3->2/4");
  }
  /*
  if(map.addStraightElement(0.0, 2.0, 3.5, 4.5, 4.0, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 2/4->0/4");
  }
  */
  if(map.addDoubleStraightElement(2.1, 3.0, 3.0, 10.0, 3.25, 3.7, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 2/3->3/4");
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 0/4->-1/3");
  }
  */
  if(map.addDoubleCurveElement(-10.0, 2.11, 2.0, 10.0, CURVE_CORNER_LR, 1.25, 1.75)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element Lower-left corner");
  }
  /*
  if(map.addStraightElement(-1.5, -0.5, 1.0, 3.0, -1.0, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element -1/3->-1/1");
  }  
  */
  if(map.addDoubleStraightElement(-10.0, 1.0, 1.0, 2.0, 0.33, 0.8, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element -1/3->-1/1");
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element -1/1->0/0");
  }
  */
  if(map.addDoubleCurveElement(-10.0, 1.0, -10.0, 1.0, CURVE_CORNER_UR, 0.225, 0.67)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element upper right corner");
  }   

    LOG_INFO("Lane map generation complete!");


}


//implement the Configure function to read ALL Properties
tResult cStanleyControl::Configure()
{
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cStanleyControl::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSampleIst;
    object_ptr<const ISample> pReadSampleSoll;

    m_properties->TriggerPropertiesReload(80); // reload the file every 2 seconds with a 25 msec timer
	stanleyGain = m_properties->GetFloat("stanley_gain");
    maxAngle = m_properties->GetFloat("max_angle");
	
    
    
    if(IS_OK(m_oVPReaderIst.GetNextSample(pReadSampleIst))) {
        auto oDecoder1 = m_VirtualPointSampleFactory.MakeDecoderFor(*pReadSampleIst);

        RETURN_IF_FAILED(oDecoder1.IsValid());

        RETURN_IF_FAILED(oDecoder1.GetElementValue(m_ddlPositionIndex.x, &carX));
        RETURN_IF_FAILED(oDecoder1.GetElementValue(m_ddlPositionIndex.y, &carY));
        RETURN_IF_FAILED(oDecoder1.GetElementValue(m_ddlPositionIndex.heading, &carHeading));
        RETURN_IF_FAILED(oDecoder1.GetElementValue(m_ddlPositionIndex.speed, &carSpeed));

        carBackPosition.x = carX;
        carBackPosition.y = carY;
        carBackPosition.h = carHeading;
        //TODO send front pos
        calculateFrontPos();

        //send front-position
        object_ptr<ISample> pSample;
        RETURN_IF_FAILED(alloc_sample(pSample, tmTimeOfTrigger));

        auto oCodec = m_VirtualPointSampleFactory.MakeCodecFor(pSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, carFrontPosition.x));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, carFrontPosition.y));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, 0));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, carSpeed));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, carFrontPosition.h));

        //LOG_INFO(cString::Format("sendPositionStruct: %.3f %.3f %.3f %.3f %.3f", f32X, f32Y,
            //f32Radius, f32Heading, f32Speed).GetPtr());

        //RETURN_NOERROR; 

        vp=map.getNormalPoint(carFrontPosition);
        //double spd=map.getSpeedAdvisory();
        LITD_map_error_t err=map.getMapState();
        if(err!=MAP_ENOERR) {
            LOG_ERROR("LaneGenerator (x=%lf; y=%lf; heading=%lf) is in invalid state: \"%s\" Resetting speed!", static_cast<double>(carFrontPosition.x),static_cast<double>(carFrontPosition.y),static_cast<double>(carFrontPosition.h),map.strerr(err).c_str());
            vp=carFrontPosition;
        }
    

        if(carSpeed > 0.0){
            // Do the Processing
            LOG_INFO("Soll x : %.2f, soll y: %.2f ", vp.x, vp.y);
            LOG_INFO("Ist x : %.2f, Ist y: %.2f ", carFrontPosition.x, carFrontPosition.y);
            calcSteeringAngle();

            if(carSteeringAngle < -M_PI/4){
                carSteeringAngle = -M_PI/4;
                LOG_INFO("Steering angle truncated to -45°!");
            } else if(carSteeringAngle > M_PI/4){
                carSteeringAngle = M_PI/4;
                LOG_INFO("Steering angle truncated to 45°!");
            }
            //calculate the mapping from -100 to +100
            mapSteeringAngle();
            LOG_INFO("Steering Value (-100 to +100) : %.2f ", carSteeringValue);
        }
    
    //TODO: check the input-type for the steering-controller
    //is this a float or a integer-value
    // in case of int, do a cast and change the output data-type
	transmitSignalValue(m_oWriter, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, carSteeringValue);
    } else {
        LOG_ERROR("Reading sample failed!!");
    }
    RETURN_NOERROR;
}
