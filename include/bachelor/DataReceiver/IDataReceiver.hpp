#ifndef BACHELOR_DATARECEIVER_IDATARECEIVER_HPP_
#define BACHELOR_DATARECEIVER_IDATARECEIVER_HPP_

#include <bachelor/IObserver.hpp>

template <typename T1>
class IDataReceiver
{
public:
    explicit IDataReceiver() = default;
    virtual ~IDataReceiver() = default;

    virtual void registerObserver(IObserver<T1> *observer) = 0;
	virtual void removeObserver(IObserver<T1> *observer) = 0;
};

#endif //BACHELOR_DATARECEIVER_IDATARECEIVER_HPP_