#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCVSTRING_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCVSTRING_HPP_

#include "PlatformRcv.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bachelor/Message/StringMessage.hpp>

class PlatformRcvString : public PlatformRcv
{
    StringMessage m_Message;
    void Callback(const std_msgs::String& msg);
public:
    PlatformRcvString(const Topic topic);
    virtual ~PlatformRcvString() = default;

    const IMessage* getMessage(void) const override;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCVSTRING_HPP_