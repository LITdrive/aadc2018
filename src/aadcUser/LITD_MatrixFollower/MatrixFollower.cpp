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
#include "MatrixFollower.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_COPENCVTEMPLATE_DATA_TRIGGERED_FILTER,
                                    "MatrixFollower_cf",
                                    cMatrixFollower,
                                    adtf::filter::pin_trigger({ "input" }));

cMatrixFollower::cMatrixFollower()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "i_BirdsEyeImg", pType);

    //Register output pins
    Register(m_oSpeedWriter, "o_Speed", pType);
    Register(m_oSteeringWriter, "o_Steering", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });

    // generating bitmask path
    m_Path = imread(PATH_TO_PATH);

}

tResult cMatrixFollower::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult cMatrixFollower::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat bvImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                   CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

            //Do the localization
            Point2i newLoc = m_locator.localize(bvImage, Point2i(x + int(vel*sin(heading)), y +int(vel*cos(heading))), SEARCH_SPACE_SIZE);
            int x_new = newLoc.x ,y_new = newLoc.y;

            // Estimate new Velocity and Heading
            int x_diff = x_new - x;
            int y_diff = y_new - y;

            vel = sqrt(x_diff*x_diff + y_diff*y_diff);
            heading += atan2(y_diff, x_diff);
            //Set new position
            x = x_new;
            y = y_new;

            //cutout new steering information
            Mat car_coord_shift = Mat::eye(3,3, CV_32F);
            car_coord_shift.at<float>(0, 2) = -x;
            car_coord_shift.at<float>(1, 2) = -y;
            // map rotation -> car rotation
            Mat car_rot = Mat::eye(3,3, CV_32F);
            Mat rot = getRotationMatrix2D(Point2f(0, 0), -heading, 1.0f);
            rot.copyTo(car_rot(Rect_<int>(0,0,1,2)));
            // car location -> picture location
            Mat offset = Mat::eye(3,3, CV_32F);
            offset.at<float>(0, 2) += CUTOUT_X / 2.f;
            offset.at<float>(1, 2) += CUTOUT_Y;
            Mat combined = offset*car_rot*car_coord_shift;
            combined = combined(Rect_<int>(0,0,1,2)); // only select the Affine Part of the Transformation
            //CutOut path for steering
            Mat m_PathCutOut;
            warpAffine(m_Path, m_PathCutOut ,combined, Size(CUTOUT_X, CUTOUT_Y), INTER_LINEAR, BORDER_REPLICATE);
            // Convert to binary
            Mat m_PathBitMask;
            threshold(m_PathCutOut, m_PathBitMask, 1, 1, THRESH_BINARY_INV);

            vector<int> mult_vec;
            for (int i = 0; i < CUTOUT_X; ++i) mult_vec.push_back(i-CUTOUT_X/2);

            Mat mult_mat = Mat(mult_vec);

            double value = sum(m_PathCut_Out*mult_mat);
            LOG_INFO(value);

        }
    }
    
    RETURN_NOERROR;
}