#ifndef BACHELOR_DATAPROTOCOL_TEMPLATE_IDATARECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_TEMPLATE_IDATARECEIVER_HPP_

#include <bachelor/Observer/IObserver.hpp>

template <typename T1, typename T2>
class IDataReceiver
{
public:
    explicit IDataReceiver() = default;
    virtual ~IDataReceiver() = default;

    virtual void registerObserver(IObserver<T1> *observer, Topics _subjTopic) = 0;
	virtual void removeObserver(IObserver<T1> *observer) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_TEMPLATE_IDATARECEIVER_HPP_