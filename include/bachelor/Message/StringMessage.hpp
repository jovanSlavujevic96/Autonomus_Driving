#ifndef BACHELOR_MESSAGE_STRINGMESSAGE_HPP_
#define BACHELOR_MESSAGE_STRINGMESSAGE_HPP_

#include "IMessage.hpp"
#include <vector>
#include <string>

class StringMessage : public IMessage
{
public:
    Topic topic;
    std::vector<std::string> text;
};

#endif //BACHELOR_MESSAGE_STRINGMESSAGE_HPP_