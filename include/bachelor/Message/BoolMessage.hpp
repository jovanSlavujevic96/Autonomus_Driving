#ifndef BACHELOR_MESSAGE_BOOLMESSAGE_HPP_
#define BACHELOR_MESSAGE_BOOLMESSAGE_HPP_

#include <bachelor/Topic.h>
#include "IMessage.hpp"

class BoolMessage : public IMessage
{
public:
    Topic topic;
    bool info;
};

#endif //BACHELOR_MESSAGE_BOOLMESSAGE_HPP_

