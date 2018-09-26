#ifndef _ZEROMQ_INTERFACE_SERVICE_HEADER_
#define _ZEROMQ_INTERFACE_SERVICE_HEADER_

#define CID_INTERACE_ZEROMQ_SERVICE "zeromq_intf.user.adtf.iid"

class IZeroMQService  : public  adtf::ucom::IObject
{

protected:
	~IZeroMQService() = default;

public:
	ADTF_IID(IZeroMQService, CID_INTERACE_ZEROMQ_SERVICE);

	virtual zmq::context_t* GetContext() = 0;
};


#endif // _ZEROMQ_INTERFACE_SERVICE_HEADER_
