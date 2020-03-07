#ifndef BACHELOR_DATAPROTOCOL_SENDER_HPP_
#define BACHELOR_DATAPROTOCOL_SENDER_HPP_

#include "ISender.hpp"

#include <memory>
#include <bachelor/Topic.h>

template <typename T1>
class Sender : 
    public ISender
{   
    class ImplDataSender;
    std::unique_ptr<ImplDataSender> m_PimplDataSender;
public:
    Sender(const Topic topicName);
    virtual ~Sender();

    void Publish(const IMessage* data) override;
};

#endif //BACHELOR_DATAPROTOCOL_SENDER_HPP_