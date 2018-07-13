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

/*!
 * Transmit signal value.
 *
 * \param [in,out]  outputPin                   The output pin.
 * \param           streamTime                  The stream time.
 * \param [in,out]  signalValueSampleFactory    The signal value sample factory.
 * \param           timeStampId                 Identifier for the time stamp.
 * \param           timeStamp                   The time stamp.
 * \param           valueId                     Identifier for the value.
 * \param           value                       The value.
 *
 * \return  Standard Result Code.
 */
tResult transmitSignalValue(cPinWriter& outputPin, tTimeStamp streamTime, adtf::mediadescription::cSampleCodecFactory& signalValueSampleFactory, tSize timeStampId, tUInt32 timeStamp, tSize valueId, tFloat32 value)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, streamTime));
    {
        auto oCodec = signalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(timeStampId, timeStamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(valueId, value));
    }

    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}

/*!
 * Transmit bool signal value.
 *
 * \param [in,out]  outputPin                       The output pin.
 * \param           streamTime                      The stream time.
 * \param [in,out]  boolSignalValueSampleFactory    The signal value sample factory.
 * \param           timeStampId                     Identifier for the time stamp.
 * \param           arduinoTimestamp                The arduino timestamp.
 * \param           valueId                         Identifier for the value.
 * \param           boolValue                       The value.
 *
 * \return  Standard Result Code.
 */
tResult transmitBoolSignalValue(cPinWriter& outputPin, tTimeStamp streamTime, adtf::mediadescription::cSampleCodecFactory& boolSignalValueSampleFactory, tSize timeStampId, tUInt32 arduinoTimestamp, tSize valueId, tBool boolValue)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, streamTime));
    {
        auto oCodec = boolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(timeStampId, arduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(valueId, boolValue));   
    }

    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;

}
