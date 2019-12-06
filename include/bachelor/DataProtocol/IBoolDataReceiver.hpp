#ifndef BACHELOR_DATAPROTOCOL_IBOOLDATARECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_IBOOLDATARECEIVER_HPP_

#include <bachelor/Observer/IBoolObserver.hpp>
#include <bachelor/Topics.h>

//Data Receiver interface
class IBoolDataReceiver
{
public:
    explicit IBoolDataReceiver() = default;
    virtual ~IBoolDataReceiver() = default;    

    virtual void registerObserver(IBoolObserver *observer, Topics _subjTopic) = 0;
	virtual void removeObserver(IBoolObserver *observer) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IBOOLDATARECEIVER_HPP_