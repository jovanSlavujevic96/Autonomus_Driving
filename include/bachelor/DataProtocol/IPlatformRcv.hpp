#ifndef BACHELOR_DATAPROTOCOL_IPLATFORMRCV_HPP_
#define BACHELOR_DATAPROTOCOL_IPLATFORMRCV_HPP_

class IObserver;
class Message;
class IMessage;

class IPlatformRcv
{
public:
    explicit IPlatformRcv() = default;
    virtual ~IPlatformRcv() = default;

    virtual void registerObserver(IObserver* observer) = 0;
	virtual void removeObserver(IObserver* observer) = 0;

    virtual const IMessage* getMessage(void) const = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IPLATFORMRCV_HPP_