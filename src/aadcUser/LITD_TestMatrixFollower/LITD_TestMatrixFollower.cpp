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
#include "LITD_TestMatrixFollower.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_COPENCVTEMPLATE_DATA_TRIGGERED_FILTER,
                                    "LITD_TestMatrixFollower_cf",
                                    cLITD_TestMatrixFollower,
                                    adtf::filter::pin_trigger({ "input" }));

double rad2grad(double x);

cLITD_TestMatrixFollower::cLITD_TestMatrixFollower()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });
    
object_ptr<IStreamType> pTypeSignalValue;
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }

    //Register output pins
    Register(m_oSteeringWriter, "steering", pTypeSignalValue);
    Register(m_oSpeedWriter, "speed", pTypeSignalValue);

    // generating bitmask path
    m_Path = imread(PATH_TO_PATH);
}

tResult cLITD_TestMatrixFollower::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult cLITD_TestMatrixFollower::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    Mat outputImage;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        //ignore 20 first images because of camer adjustment
        if(imgcnt<30){imgcnt++;continue;}
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                   CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

            //Do the image processing and copy to destination image buffer
            //Canny(inputImage, outputImage, 100, 200);// Detect Edges
            Mat bvImage;
            cvtColor(inputImage, bvImage, COLOR_BGR2RGB);
            //Do the localization
            Point2i newLoc = m_locator.localize(bvImage, heading, Point2i(x + int(vel*sin(heading)), y + int(vel*cos(heading))), SEARCH_SPACE_SIZE);
            int x_new = newLoc.x ,y_new = newLoc.y;

            //cout << endl << endl << x_new << "\t" << y_new << endl << endl;
            
            // Estimate new Velocity and Heading
            int x_diff = x_new - x;
            int y_diff = y_new - y;
            vel = 0.8*vel + 0.2*sqrt(x_diff*x_diff + y_diff*y_diff);
            if(vel > VELOCITY_DEADBAND){
                heading = heading*0.9 + 0.1*atan2(y_diff, x_diff);
                //Set new position
                x = x_new;
                y = y_new;
            }
            LOG_INFO("%d\t %d\t %lf\t %lf",x_new, y_new, vel, heading);
            //cutout new steering information
            Mat car_coord_shift = Mat::eye(3,3, CV_32F);
            car_coord_shift.at<float>(0, 2) = -x;
            car_coord_shift.at<float>(1, 2) = -y;
            // map rotation -> car rotation
            Mat car_rot = Mat::eye(3,3, CV_32F);
            Mat rot = getRotationMatrix2D(Point2f(0, 0), rad2grad(-heading), 1.0f);
            rot(Rect(0,0,2,2)).copyTo(car_rot(Rect(0,0,2,2)));
            // car location -> picture location
            Mat offset = Mat::eye(3,3, CV_32F);
            offset.at<float>(0, 2) += CUTOUT_X / 2.f;
            offset.at<float>(1, 2) += CUTOUT_Y;
            Mat combined = offset*car_rot*car_coord_shift;
            combined = combined(Rect(0,0,3,2)).clone(); // only select the Affine Part of the Transformation
            //CutOut path for steering
            Mat m_PathCutOut;
            warpAffine(m_Path, m_PathCutOut ,combined, Size(CUTOUT_X, CUTOUT_Y), INTER_LINEAR, BORDER_REPLICATE);
            // Convert to binary
            Mat m_PathBitMask;
            threshold(m_PathCutOut, m_PathBitMask, 1, 1, THRESH_BINARY_INV);

            vector<int> mult_vec;
            for (int i = 0; i < CUTOUT_X; ++i) mult_vec.push_back(i-CUTOUT_X/2);

            Mat mult_mat = Mat(mult_vec);
            mult_mat.convertTo(mult_mat, CV_64F);
            m_PathBitMask.convertTo(m_PathBitMask, CV_64F);
            double value = 0.0;
            for(int i = 0; i < m_PathBitMask.size[0]; i++){
                for(int j = 0; j < m_PathBitMask.size[1]; j++){
                    value += mult_vec[i]*m_PathBitMask.at<double>(i,j);
                }
            }
            value = value / m_PathBitMask.size[1]; //Normalize to rowcount
            value = value / ((CUTOUT_X/2.0 + 1)*CUTOUT_X/4); // Normalize to max
            /*Scalar value = sum(m_PathBitMask*mult_mat.t());*/
            LOG_INFO("%d", int(value*90));

            transmitSignalValue(m_oSteeringWriter, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
	                    m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, float(int(value*90)));
            transmitSignalValue(m_oSpeedWriter, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
	                    m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, 12);
        }
    }
    
    RETURN_NOERROR;
}

double rad2grad(double x){
    return x*M_PI/180.0;
}