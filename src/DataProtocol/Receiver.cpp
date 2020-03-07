#include <bachelor/DataProtocol/Receiver.hpp>
#include <bachelor/DataProtocol/IPlatformRcv.hpp>

//main class impl
Receiver::Receiver(std::unique_ptr<IPlatformRcv> receiver) :
    m_Pimpl{std::move(receiver)}
{

}

Receiver::~Receiver() = default;

void Receiver::registerObserver(IObserver* observer)
{
    m_Pimpl->registerObserver(observer);
}

void Receiver::removeObserver(IObserver* observer)
{
    m_Pimpl->removeObserver(observer);
}
