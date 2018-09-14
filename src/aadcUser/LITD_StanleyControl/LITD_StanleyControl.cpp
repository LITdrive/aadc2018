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

/* notes to check
[] actual speed has to be in car-struct
[] is the given point normal to car position?

*/


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LITD_StanleyControl",
    cStanleyControl,
    adtf::filter::pin_trigger({"input"}));


void cStanleyControl::calcSteeringAngle(){
    double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
    Vector2d diff = vp.getVector2d() - carPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vp.h))> 4.712 ){
        sign = -1;
    }

    //calc normal distance of tangent to car (e)
    double e = (vp.getVector2d() - carPosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    double theta_c =  wrapTo2Pi(vp.h) - wrapTo2Pi(carPosition.h);

    //calc steering-angle with stanley-approach
    carSteeringAngle = theta_c + atan2(stanleyGain*e, carSpeed);

    //Debug Messages
    /*std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    std::cout << "Steering Angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;*/

}
cStanleyControl::cStanleyControl()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeVirtualPoint;
    object_ptr<IStreamType> pTypeTemplateData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVirtualPoint", pTypeVirtualPoint, m_VirtualPointSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64x"), m_ddlVirtualPointId.f64x));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64y"), m_ddlVirtualPointId.f64y));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Heading"), m_ddlVirtualPointId.f64Heading));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Speed"), m_ddlVirtualPointId.f64Speed));
    } else {
        LOG_INFO("No mediadescription for tVirtualPoint found!");
    }
    //Register input pin
    Register(m_oVPReaderIst, "inVirtualPoint", pTypeVirtualPoint);
    Register(m_oVPReaderSoll, "inVirtualPoint", pTypeVirtualPoint);
    Register(m_oWriter, "output", pTypeTemplateData);
}


//implement the Configure function to read ALL Properties
tResult cStanleyControl::Configure()
{
    RETURN_NOERROR;
}

tResult cStanleyControl::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSampleIst;
    object_ptr<const ISample> pReadSampleSoll;
    
    
    if(IS_OK(m_oVPReaderIst.GetNextSample(pReadSampleIst))) {
        auto oDecoder = m_VirtualPointSampleFactory.MakeDecoderFor(*pReadSampleIst);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64x, &carX));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64y, &carY));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Heading, &carHeading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Speed, &carSpeed));

        carPosition.x = carX;
        carPosition.y = carY;
    }
    if(IS_OK(m_oVPReaderSoll.GetNextSample(pReadSampleSoll))) {
        auto oDecoder = m_VirtualPointSampleFactory.MakeDecoderFor(*pReadSampleSoll);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64x, &sollX));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64y, &sollY));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Heading, &sollHeading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Speed, &sollSpeed));

        vp.x = sollX;
        vp.y = sollY;
    }

    // Do the Processing
    calcSteeringAngle();

    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_VirtualPointSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlStanleyOutputDataId.f64Value, carSteeringAngle));

    }
    m_oWriter << pWriteSample << flush << trigger;
    
    RETURN_NOERROR;
}