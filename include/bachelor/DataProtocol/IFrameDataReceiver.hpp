#ifndef BACHELOR_DATAPROTOCOL_IFRAMEDATARECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_IFRAMEDATARECEIVER_HPP_

#include <bachelor/Observer/IImageObserver.hpp>

class IFrameDataReceiver
{
public:
    explicit IFrameDataReceiver() = default; 
	virtual ~IFrameDataReceiver() = default;
	
	virtual void registerObserver(IImageObserver *observer, Topics _subjTopic) = 0;
	virtual void removeObserver(IImageObserver *observer) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IFRAMEDATARECEIVER_HPP_