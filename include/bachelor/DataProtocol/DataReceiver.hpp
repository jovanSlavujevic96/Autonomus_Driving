#ifndef BACHELOR_DATAPROTOCOL_DATARECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_DATARECEIVER_HPP_

#include "IDataReceiver.hpp"
#include <memory>

template <typename T1, typename T2>
class DataReceiver : public IDataReceiver<T1,T2>
{
    class ImplDataReceiver;
    std::unique_ptr<ImplDataReceiver> m_PimplDataReceiver;
public:
    DataReceiver(const Topics _topicName);
    virtual ~DataReceiver();

    virtual void registerObserver(IObserver<T1> *observer, Topics _subjTopic) override;
	virtual void removeObserver(IObserver<T1> *observer) override;
};

#endif //BACHELOR_DATAPROTOCOL_DATARECEIVER_HPP_