#ifndef BACHELOR_DATASENDER_DATASENDER_HPP_
#define BACHELOR_DATASENDER_DATASENDER_HPP_

#include "IDataSender.hpp"

#include <memory>
#include <bachelor/Topic.h>

template <typename T1>
class DataSender : 
    public IDataSender <T1>
{   
    class ImplDataSender;
    std::unique_ptr<ImplDataSender> m_PimplDataSender;
    Topic m_Topic;
public:
    DataSender(const Topic topicName);
    virtual ~DataSender();

    void Publish(const T1& data) override;
    Topic getTopic(void) const override;
};

#endif //BACHELOR_DATASENDER_DATASENDER_HPP_