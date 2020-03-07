#include <bachelor/DataProtocol/PlatformRcv.hpp>
#include <bachelor/IObserver.hpp>

void PlatformRcv::notifyObservers(void)
{
    for (IObserver* observer : m_Observers)  
    // notify all observers
    {
        observer->update(this);
    }
}

void PlatformRcv::registerObserver(IObserver* observer) 
{
    PlatformRcv::m_Observers.push_back(observer);
}

void PlatformRcv::removeObserver(IObserver* observer) 
{
    auto iterator = std::find(m_Observers.begin(), m_Observers.end(), observer);
    if (iterator != m_Observers.end()) // observer found
    {
        PlatformRcv::m_Observers.erase(iterator); // remove the observer
    }
}   