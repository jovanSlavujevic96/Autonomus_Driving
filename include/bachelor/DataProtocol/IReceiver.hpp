#ifndef BACHELOR_DATAPROTOCOL_IRECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_IRECEIVER_HPP_

#include <bachelor/IObserver.hpp>

class IReceiver
{
public:
    explicit IReceiver() = default;
    virtual ~IReceiver() = default;

    virtual void registerObserver(IObserver* observer) = 0;
	virtual void removeObserver(IObserver* observer) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IRECEIVER_HPP_