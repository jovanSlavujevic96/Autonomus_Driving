#ifndef BACHELOR_DATAPROTOCOL_DATASENDER_HPP_
#define BACHELOR_DATAPROTOCOL_DATASENDER_HPP_

#include "IDataSender.hpp"

#include <memory>
#include <bachelor/Topics.h>

template <typename T1, typename T2>
class DataSender : public IDataSender <T1,T2>
{   
    class ImplDataSender;
    std::unique_ptr<ImplDataSender> m_PimplDataSender;
public:
    DataSender(const Topics _topicName);
    virtual ~DataSender();

    virtual void Publish(T1 _data) override;
};

#endif //BACHELOR_DATAPROTOCOL_DATASENDER_HPP_