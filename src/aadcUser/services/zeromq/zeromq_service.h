#ifndef _ZEROMQ_SERVICE_HEADER_
#define _ZEROMQ_SERVICE_HEADER_

#define CID_ZEROMQ_SERVICE "zeromq.service.adtf.cid"

class cZeroMQService : public object<cADTFService, IZeroMQService>
{
public:
	ADTF_CLASS_ID_NAME(cZeroMQService, CID_ZEROMQ_SERVICE, "ZeroMQ Service");

	ADTF_CLASS_DEPENDENCIES(PROVIDE_INTERFACE(IZeroMQService));

public:
	///CTOR
	cZeroMQService() = default;

public: // overrides cService
	/// override this virtual Callback to initialize the service
	tResult ServiceInit() override;
	/// override this virtual Callback to shutdown the service
	tResult ServiceShutdown() override;

public: // implements IZeroMQService
	zmq::context_t* GetContext() override;

private:
	/*! Holds the context pointer  */
	zmq::context_t* m_zeromq_ctx = nullptr;
};

#endif // _ZEROMQ_SERVICE_HEADER_
