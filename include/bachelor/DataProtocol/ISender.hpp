#ifndef BACHELOR_DATAPROTOCOL_ISENDER_HPP_
#define BACHELOR_DATAPROTOCOL_ISENDER_HPP_

#include <bachelor/Topic.h>

class IMessage;

class ISender
{
public:
    explicit ISender() = default;
    virtual ~ISender() = default;

    virtual void Publish(const IMessage* data) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_ISENDER_HPP_