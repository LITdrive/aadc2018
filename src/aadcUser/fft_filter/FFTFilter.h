#pragma once
#define CID_FFT_DATA_TRIGGERED_FILTER "fft.filter.user.aadc.cid"
#include "stdafx.h"

#define REAL 0
#define IMAG 1
#define FREQUENCY_SAMPLING 44100															// Hz
#define WINDOW_LENGTH 1000																	// Window Length
#define NUMBER_OF_FFT_INPUT_SAMPLES (tInt)(pow(2, ceil(std::log(WINDOW_LENGTH) / std::log(2))))
#define NUMBER_OF_FFT_OUTPUT_SAMPLES (tInt)(NUMBER_OF_FFT_INPUT_SAMPLES/2 + 1)

using namespace adtf_util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;

class cFFTFilter : public cFilterBase
{
public:
	ADTF_CLASS_ID_NAME(cFFTFilter, CID_FFT_DATA_TRIGGERED_FILTER, "Data Triggered FFT Filter");

private:
	///Reader to read data samples from an InPin
	cPinReader m_oReader;

	///Writer to write data samples to an OutPin
	cPinWriter m_oWriter;

	///TimeStamps to control logging rate
	/* tTimeStamp m_tmLogPeriod = 1000000;
	tTimeStamp m_tmTimeLastLog = 0; */

	runnable<>		m_oRunnable;

public:
	///C-TOR
	cFFTFilter();

	/// D-TOR
	virtual ~cFFTFilter() = default;

	/**
	* Overwrites the Init
	* Create pins, runners and connect them
	*/
	tResult Init(tInitStage eInitStage);

	/**
	* Read data samples from InPins, modify data and write the new data sample to the OutPin
	*/
	tResult RunTrigger(tTimeStamp tmTimeofActivation);

	void writeMatToPin(adtf::streaming::cSampleWriter & writer, const cv::Mat & outputImage, tTimeStamp streamTime);
};