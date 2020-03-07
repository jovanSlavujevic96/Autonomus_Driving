#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCVBOOL_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCVBOOL_HPP_

#include "PlatformRcv.hpp"
#include <std_msgs/Bool.h>
#include <bachelor/Message/BoolMessage.hpp>

class PlatformRcvBool : public PlatformRcv
{
    BoolMessage m_Message;
    void Callback(const std_msgs::Bool& msg);

public:
    PlatformRcvBool(const Topic topic);
    virtual ~PlatformRcvBool() = default;
    
    const IMessage* getMessage(void) const override;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCVBOOL_HPP_