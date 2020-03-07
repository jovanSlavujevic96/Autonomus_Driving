#ifndef BACHELOR_DATAPROTOCOL_RECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_RECEIVER_HPP_

#include "IReceiver.hpp"
#include <memory>

class IPlatformRcv;

class Receiver : 
    public IReceiver
{
    std::unique_ptr<IPlatformRcv> m_Pimpl;
public:
    Receiver(std::unique_ptr<IPlatformRcv> receiver);
    virtual ~Receiver();

    void registerObserver(IObserver* observer) override;
	void removeObserver(IObserver* observer) override;
};

#endif //BACHELOR_DATAPROTOCOL_RECEIVER_HPP_