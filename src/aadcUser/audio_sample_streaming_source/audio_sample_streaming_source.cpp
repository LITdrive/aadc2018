// audio_sample_streaming_source.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"
#include "audio_sample_streaming_source.h"

#define FREQUENCY_SAMPLING 44100															// Hz
#define FREQUENCY_SIGNAL 1000																// Hz
#define BUFFER_SIZE 44000

ADTF_PLUGIN("Audio Sampling Plugin", cAudioSampleStreamingSource);

tInt sample_index = 0;
/* tInt i;
tInt err;
tFloat64 buf[BUFFER_SIZE];
snd_pcm_t *capture_handle;
snd_pcm_hw_params_t *hw_params; */

cAudioSampleStreamingSource::cAudioSampleStreamingSource()
{
	RegisterPropertyVariable("sampling_time", m_nSpeed);
}

tResult cAudioSampleStreamingSource::Construct()
{
	LOG_INFO("Construct of audio sample streaming source was called");

	RETURN_IF_FAILED(cSampleStreamingSource::Construct());

	//ALSA Parameter setzen
	/*if ((err = snd_pcm_open(&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0)) < 0)
	{
		LOG_INFO("Cannot open audio device");
	}

	if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0)
	{
		LOG_INFO("cannot allocate hardware parameter structure");
	}

	if ((err = snd_pcm_hw_params_any(capture_handle, hw_params)) < 0)
	{
		LOG_INFO("cannot initialize hardware parameter structure");
	}

	if ((err = snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
	{
		LOG_INFO("cannot set access type");
	}

	if ((err = snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0)
	{
		LOG_INFO("cannot set sample format");
	}

	if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, 44100, 0)) < 0)
	{
		LOG_INOF("cannot set sample rate");
	}

	if ((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1)) < 0)
	{
		LOG_INFO("cannot set channel count");
	}

	if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0)
	{
		LOG_INFO("cannot set parameters");
	}

	snd_pcm_hw_params_free(hw_params);

	if ((err = snd_pcm_prepare(capture_handle)) < 0)
	{
		LOG_INFO("cannot prepare audio interface for use");
	}*/

	// Create a stream type with the given media description.
	// To set values inside the media description we let
	// the method create a codec factory for us
	object_ptr<IStreamType> pTypeSimple;
	create_adtf_default_stream_type("tSimple", strAudioSampleStructDescription, pTypeSimple, m_oCodecFactorySimple);

	RETURN_IF_FAILED(create_pin(*this, m_oOutputWriter, "sampleOut", pTypeSimple));

	RETURN_NOERROR;
}

tResult cAudioSampleStreamingSource::StartStreaming()
{
	RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());

	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

	// This timer calls repeatedly our function that contains the character generation logic
	m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::sampling_timer"), m_nSpeed, 0, &cAudioSampleStreamingSource::TimerFunc, this);

	if (!m_oTimer.Stoppable())
	{
		RETURN_ERROR_DESC(ERR_UNEXPECTED, "Audio Sample Streaming Source Unable to create kernel timer");
	}

	LOG_INFO("Writing audio samples...");

	RETURN_NOERROR;
}

tResult cAudioSampleStreamingSource::StopStreaming()
{
	m_oTimer.Stop();
	return cSampleStreamingSource::StopStreaming();
}

tResult cAudioSampleStreamingSource::RequestPin(const tChar * strName, const iobject_ptr<const IStreamType>& pType, iobject_ptr<IOutPin>& pOutPin)
{
	LOG_INFO("RequestPin was called, but it is not implemented for audio sample streaming source!");

	RETURN_NOERROR;
}

tVoid cAudioSampleStreamingSource::TimerFunc()
{
	tFloat64 audio_sample = cos(2 * M_PI * FREQUENCY_SIGNAL / FREQUENCY_SAMPLING * sample_index);

	// Non-interleaved reading
	/*if((err = snd_pcm_readn(capture_handle, buf, BUFFER_SIZE)) != BUFFER_SIZE)
	{
			LOG_INFO("read from audio interface failed");		
	}*/
	

	if(sample_index < ceil(FREQUENCY_SAMPLING / FREQUENCY_SIGNAL))
	{
		sample_index++;
	}

	else 
	{
		sample_index = 0;
	}

	// We log the generated value to the console just until we have a streaming sink that
	// can handle it for us.
	LOG_INFO(cString::Format("Reading audio sample: %d", audio_sample));

	// Create a new sample
	object_ptr<ISample> pWriteSample;
	tResult res = alloc_sample(pWriteSample, m_pClock->GetStreamTime());
	if (res.IsFailed()) {
		LOG_INFO("Could not allocate RAM for sample, sorry!");
		return;
	}

	// Use the DDL and decoder to write data into the sample
	{
		auto oCodec = m_oCodecFactorySimple.MakeCodecFor(pWriteSample);

		// Find the index of the xml element we want to write into
		res = ddl::access_element::find_index(m_oCodecFactorySimple, "cValue", m_nValueElementIndex);

		if (res.IsFailed()) {
			LOG_INFO("Could not find element >>cValue<< in DDL, sorry!");
			return;
		}

		tFloat64 payload = audio_sample;
		res = oCodec.SetElementValue(m_nValueElementIndex, payload);

		if (res.IsFailed()) {
			LOG_INFO("Could not set cValue in DDL, sorry!");
			return;
		}
		// The sample buffer lock is released in the destructor of oCodec
	}
	m_oOutputWriter << pWriteSample << trigger;
}

