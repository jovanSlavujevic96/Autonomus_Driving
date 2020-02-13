#ifndef BACHELOR_DATARECEIVER_DATARECEIVER_HPP_
#define BACHELOR_DATARECEIVER_DATARECEIVER_HPP_

#include "IDataReceiver.hpp"
#include <memory>

template <typename T1>
class DataReceiver : public IDataReceiver<T1>
{
    class ImplDataReceiver;
    std::unique_ptr<ImplDataReceiver> m_PimplDataReceiver;
public:
    DataReceiver(const Topics _topicName);
    virtual ~DataReceiver();

    void registerObserver(IObserver<T1> *observer) override;
	void removeObserver(IObserver<T1> *observer) override;
};

#endif //BACHELOR_DATARECEIVER_DATARECEIVER_HPP_