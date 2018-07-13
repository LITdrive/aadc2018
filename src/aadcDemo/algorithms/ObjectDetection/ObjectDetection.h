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

#define CID_OBJECT_DETECTION_FILTER "object_detection.filter.demo.aadc.cid"
#define LABEL_OBJECT_DETECTION_FILTER  "Object Detection"

/*! This is the main class for the filter for object detection with caffe. */
class cObjectDetection : public cTriggerFunction
{
private:
    /*! The writer for output image */
    cPinWriter m_oWriter;
    /*! The reader for image data*/
    cPinReader m_oReader;
    /*! The classification writer */
    cPinWriter m_oClassificationWriter;

    struct
    {
        /*! Identifier for the class name */
        tSize classNameId;
        /*! Identifier for the class */
        tSize classIdId;
        /*! Identifier for the prob value */
        tSize probValueId;

    }
    /*! the ids for ddl access*/
    m_tClassificationIds;

    /*! The codec factory */
    cSampleCodecFactory m_oCodecFactory;

    /*! The current stream imageformat */
    adtf::streaming::tStreamImageFormat m_sCurrentFormat;

    /*! The caffe model net */
    Net m_oNet;

    /*! List of names of the class */
    std::vector<cString> m_classNames;

    /*! The open cl flag */
    property_variable<tBool> m_bOpenCL = tTrue;

    /*! The prototype text property */
    property_variable<cFilename> m_strProtoTxt = cFilename("bvlc_googlenet.prototxt");

    /*! The model property */
    property_variable<cFilename> m_strModel = cFilename("bvlc_googlenet.caffemodel");

    /*! List of names of the class property */
    property_variable<cFilename> m_strClassNames = cFilename("synset_words.txt");

    /*!
     * Gets maximum class.
     *
     * \param           probBlob    The prob BLOB.
     * \param [in,out]  classId     Identifier for the class.
     * \param [in,out]  classProb   The class prob.
     */
    static void getMaxClass(const Mat& probBlob, tUInt64& classId, double& classProb);

    /*!
     * Searches for the first maximum.
     *
     * \param [in,out]  probMat     The prob matrix.
     * \param [in,out]  classId     Identifier for the class.
     * \param [in,out]  classProb   The class prob.
     */
    static void findMax(Mat& probMat, tUInt64& classId, double& classProb);
    /*!
     * Reads class names from file
     *
     * \param   filename    Filename of the file.
     *
     * \return  The class names.
     */
    std::vector<cString> readClassNames(const char *filename);

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:
    /*! Default constructor. */
    cObjectDetection();

    tResult Configure() override;

    /*!
     * Process the given trigger.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger);

};