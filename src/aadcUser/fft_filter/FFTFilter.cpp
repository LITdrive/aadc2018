// fft_filter.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"
#include "FFTFilter.h"

///Implementation of the Plugin
///The Code behind the Macro creates a plugin and the Main Entries to the plugin DLL or shared object
///the cTutorialFilter will be available through the plugins class_factor !
ADTF_PLUGIN("ADTF Data Triggered FFT Filter Plugin", cFFTFilter);

///Media description for the tutorial data samples
const tChar* strTutorialStructDescription =
"<struct alignment=\"4\" name=\"tTutorial\" version=\"2\">"
"<element name=\"fResult\" type=\"tFloat64\" arraysize=\"1\">"
"<deserialized alignment=\"4\"/>"
"<serialized byteorder=\"LE\" bytepos=\"0\"/>"
"</element>"
"</struct>";

tInt64 sample_index = 0;
tFloat64 *fft_in;
fftw_complex *fft_out;
//tFloat64 fft_out_abs[10];
tFloat64* fft_out_abs = new tFloat64 [NUMBER_OF_FFT_OUTPUT_SAMPLES];
Mat fft_out_abs_vec;
tFloat64* frequency = new tFloat64 [NUMBER_OF_FFT_OUTPUT_SAMPLES];
fftw_plan fft_plan;

///C-TOR that maps the local m_oRunnable to the RunTrigger of cTutorialDataCalculator
cFFTFilter::cFFTFilter() : m_oRunnable([this](tTimeStamp tmTime) -> tResult {return RunTrigger(tmTime); })
{
	fft_in = (tFloat64*)fftw_malloc(sizeof(tFloat64) * NUMBER_OF_FFT_INPUT_SAMPLES);
	fft_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * NUMBER_OF_FFT_OUTPUT_SAMPLES);
	
	for (tInt i = 0; i < NUMBER_OF_FFT_OUTPUT_SAMPLES; i++)
	{
		frequency[i] = FREQUENCY_SAMPLING / NUMBER_OF_FFT_INPUT_SAMPLES*i;
	}

	fft_plan = fftw_plan_dft_r2c_1d(NUMBER_OF_FFT_INPUT_SAMPLES, fft_in, fft_out, FFTW_MEASURE);
}

///This function can be executed in various stages
///StageFirst:			Create and connect Pins and Runners
///StagePreConnect:
///StagePostConnect:
tResult cFFTFilter::Init(tInitStage eInitStage)
{
	RETURN_IF_FAILED(cFilterBase::Init(eInitStage));
	if (eInitStage == tInitStage::StageFirst)
	{
		//===================================================================
		// Init StageFirst is to define RuntimeBehaviour and Pins
		//===================================================================

		//Create a Stream Type using the strTutorialStructDescription
		object_ptr<IStreamType> pStreamType;
		create_adtf_default_stream_type("tTutorial", strTutorialStructDescription, pStreamType);

		//Create the Pins for the Readers
		RETURN_IF_FAILED(create_pin(*this, m_oReader, "in1", pStreamType));
		// RETURN_IF_FAILED(create_pin(*this, m_oReader2, "in2", pStreamType));

		//and for the Writer
		RETURN_IF_FAILED(create_pin(*this, m_oWriter, "out1", pStreamType));

		// create a runner which can be called (the runnable m_oRunnable will forward to RunTrigger)
		RETURN_IF_FAILED(RegisterRunner("tutorial_runner", m_oRunnable));

		// if the runner is called the "out1" pin will be triggered, this will forward the runner call to "out1"!
		// (was implicit connected in ADTF 2 ... now this is explicit !
		RETURN_IF_FAILED(create_inner_pipe(*this, { "tutorial_runner", "out1" }));

		// if you want to have a default DataTrigger on myrunner, define it
		// this case will trigger the myrunner if data were received on "in1"
		// (was the IPinEventSink within ADTF 2... now this must be defined explicitly via InnerPipe !
		RETURN_IF_FAILED(create_inner_pipe(*this, { "in1", "tutorial_runner" }));


	}
	else if (eInitStage == tInitStage::StagePreConnect)
	{
		//===================================================================
		// Init StagePreConnect (StageNormal) is i.e. to open devices etc.
		//===================================================================
	}
	else if (eInitStage == tInitStage::StagePostConnect)
	{
		//============================================================
		// Init StagePostConnect (StageGraphReady) is discover
		//     Stream Type of the readers obtained while connection
		//============================================================
	}

	RETURN_NOERROR;
}

///This function will be executed each time a trigger occurs
///It reads data samples, modifies the data and forwards new data
tResult cFFTFilter::RunTrigger(tTimeStamp tmTimeofActivation)
{
	object_ptr<const ISample> pReadSample;
	object_ptr<ISample> pSampleToWrite;
	//this will empty the Reader queue and return the last sample received.
	//If no sample was during the time between the last execution of Process the "old sample" is return.
	if (IS_OK(m_oReader.GetLastSample(pReadSample)))
	{
		RETURN_IF_FAILED(read_from_sample(*pReadSample, fft_in[sample_index]));
		sample_index++;

		if (sample_index >= WINDOW_LENGTH)
		{
			for(tInt i = sample_index; i < NUMBER_OF_FFT_INPUT_SAMPLES; i++)
			{
				fft_in[i] = 0;
			}

			// FFT berechnen
			fftw_execute(fft_plan);
			
			// Berechnung Betragsgang (reellwertig) aus komplexwertigem Spektrum 
			for(tInt i = 0; i < NUMBER_OF_FFT_OUTPUT_SAMPLES; i++)
			{
				if (i > 0 && i < NUMBER_OF_FFT_OUTPUT_SAMPLES - 1)
				{
					fft_out_abs[i] = 2*sqrt(pow(fft_out[i][REAL] / WINDOW_LENGTH, 2) + pow(fft_out[i][IMAG] / WINDOW_LENGTH, 2));
				}

				else 
				{
					fft_out_abs[i] = sqrt(pow(fft_out[i][REAL] / WINDOW_LENGTH, 2) + pow(fft_out[i][IMAG] / WINDOW_LENGTH, 2));
				}
			}

			// Write
			fft_out_abs_vec = Mat(1, NUMBER_OF_FFT_OUTPUT_SAMPLES, CV_64F, fft_out_abs);
			writeMatToPin(m_oWriter, fft_out_abs_vec, pReadSample->GetTime());
		
			sample_index = 0;
		}

	}

	RETURN_NOERROR;
}

void cFFTFilter::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
{
	//create write buffer
	object_ptr<ISample> pWriteSample;

	if (IS_OK(alloc_sample(pWriteSample, streamTime)))
	{
		object_ptr_locked<ISampleBuffer> pWriteBuffer;
		// * sizeof(tFloat64) ?
		if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels())))
		{
			pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*)outputImage.data,
				outputImage.cols * outputImage.rows * outputImage.channels()));
		}
	}

	writer << pWriteSample << flush << trigger;
}

