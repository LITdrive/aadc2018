#include "stdafx.h"
#include "zeromq_service_intf.h"
#include "zeromq_service.h"

///The ADTF Plugin Macro will add the code of a IPlugin implementation
///Your are only allowed to use it once in a binary 
///It will also add shared-object entries to your Binary
ADTF_PLUGIN("ZeroMQ Service", cZeroMQService);

tResult cZeroMQService::ServiceInit()
{
	m_zeromq_ctx = new zmq::context_t(1);
	LOG_INFO("Initialized ZeroMQ context.");

	RETURN_NOERROR;
}

tResult cZeroMQService::ServiceShutdown()
{
	m_zeromq_ctx->close();
	m_zeromq_ctx = nullptr;

	RETURN_NOERROR;
}

zmq::context_t* cZeroMQService::GetContext()
{
	return m_zeromq_ctx;
}
