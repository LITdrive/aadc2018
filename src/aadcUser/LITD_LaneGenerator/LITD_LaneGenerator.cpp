﻿/*********************************************************************
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
#include "LITD_LaneGenerator.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LANE_GENERATOR_DATA_TRIGGERED_FILTER,
                                    "LITD_LaneGenerator",
                                    LITD_LaneGenerator,
                                    adtf::filter::pin_trigger({ "input" }));

LITD_LaneGenerator::LITD_LaneGenerator()
{
    LOG_INFO("Initializing LITD_LaneGenerator");
    object_ptr<IStreamType> pTypeVirtualPoint;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVirtualPoint", pTypeVirtualPoint, m_VirtualPointSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64x"), m_ddlVirtualPointId.f64x));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64y"), m_ddlVirtualPointId.f64y));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Heading"), m_ddlVirtualPointId.f64Heading));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Speed"), m_ddlVirtualPointId.f64Speed));
    } else {
        LOG_ERROR("No mediadescription for tVirtualPoint found!");
    }

    LOG_INFO("Generating lane map...");

   //Vertical straight between x 0->2
  /*
  if(map.addStraightElement(0.0, 2.0, -0.5, 0.5, 0.0, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 0/0->2/0");
  }
  */
  if(map.addDoubleStraightElement(0.0, 2.0, -0.5, 0.5, -0.2, 0.2, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 0/0->2/0");
  }
  /*
  if(map.addCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 2/0->3/1");
  }
  */
  if(map.addDoubleCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 0.8, 1.2)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 2/0->3/1");
  }
  /*
  if(map.addStraightElement(2.5, 3.5, 1.0, 3.0, 3.0, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 3/1->3/3");
  }
  */

  if(map.addDoubleStraightElement(2.5, 3.5, 1.0, 3.0, 2.8, 3.2, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 3/1->3/3");
  }
  /*
  if(map.addCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 3/3->2/4");
  }
  */
  if(map.addDoubleCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 0.8, 1.2)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 3/3->2/4");
  }
  /*
  if(map.addStraightElement(0.0, 2.0, 3.5, 4.5, 4.0, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 2/4->0/4");
  }
  */
  if(map.addDoubleStraightElement(0.0, 2.0, 3.5, 4.5, 3.8, 4.2, false)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element 2/4->0/4");
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 0/4->-1/3");
  }
  */
  if(map.addDoubleCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 0.8, 1.2)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element 0/4->-1/3");
  }
  /*
  if(map.addStraightElement(-1.5, -0.5, 1.0, 3.0, -1.0, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element -1/3->-1/1");
  }  
  */
  if(map.addDoubleStraightElement(-1.5, -0.5, 1.0, 3.0, -1.2, -0.8, true)!=MAP_ENOERR) {
    LOG_ERROR("Error adding straight element -1/3->-1/1");
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 1.0)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element -1/1->0/0");
  }
  */
  if(map.addDoubleCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 0.8, 1.2)!=MAP_ENOERR) {
    LOG_ERROR("Error adding curve element -1/1->0/0");
  }   

    LOG_INFO("Lane map generation complete!");

    //Register input pin
    Register(m_oVPReader, "inVirtualPoint", pTypeVirtualPoint);

    //Register output pin
    Register(m_oVPWriter, "outVirtualPoint", pTypeVirtualPoint);

}

tResult LITD_LaneGenerator::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult LITD_LaneGenerator::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample, pVPReadSample;

    RETURN_IF_FAILED_DESC(m_oVPReader.GetNextSample(pVPReadSample), "GetNextSample failed!");
    auto oDecoder = m_VirtualPointSampleFactory.MakeDecoderFor(*pVPReadSample);

    RETURN_IF_FAILED_DESC(oDecoder.IsValid(), "oDecoder isValid failed!");

    RETURN_IF_FAILED_DESC(oDecoder.GetElementValue(m_ddlVirtualPointId.f64x, &x), "GetElementValue for x failed!");
    RETURN_IF_FAILED_DESC(oDecoder.GetElementValue(m_ddlVirtualPointId.f64y, &y), "GetElementValue for y failed!");
    RETURN_IF_FAILED_DESC(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Heading, &heading), "GetElementValue for heading failed!");
    RETURN_IF_FAILED_DESC(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Speed, &speed), "GetElementValue for speed failed!");


    


    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED_DESC(alloc_sample(pWriteSample, m_pClock->GetStreamTime()), "alloc_sample failed!");
    auto oCodec = m_VirtualPointSampleFactory.MakeCodecFor(pWriteSample);

    RETURN_IF_FAILED_DESC(oCodec.SetElementValue(m_ddlVirtualPointId.f64x, x), "SetElementValue for x failed!");
    RETURN_IF_FAILED_DESC(oCodec.SetElementValue(m_ddlVirtualPointId.f64y, y), "SetElementValue for x failed!");
    RETURN_IF_FAILED_DESC(oCodec.SetElementValue(m_ddlVirtualPointId.f64Heading, heading), "SetElementValue for heading failed!");
    RETURN_IF_FAILED_DESC(oCodec.SetElementValue(m_ddlVirtualPointId.f64Speed, speed), "SetElementValue for speed failed!");

    m_oVPWriter << pWriteSample << flush << trigger;
    
    RETURN_NOERROR;
}