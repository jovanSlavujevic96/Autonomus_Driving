#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCVLOG_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCVLOG_HPP_

#include "PlatformRcv.hpp"
#include <ros/ros.h>
#include <bachelor/Log.h>
#include <bachelor/Message/StringMessage.hpp>

class PlatformRcvLog : public PlatformRcv
{
    StringMessage m_Message;
    void Callback(const bachelor::Log& msg);

public:
    PlatformRcvLog(const Topic topic);
    virtual ~PlatformRcvLog() = default;

    const IMessage* getMessage(void) const override;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCVLOG_HPP_