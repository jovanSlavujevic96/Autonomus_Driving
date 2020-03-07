#include <bachelor/DataProtocol/PlatformRcvBool.hpp>
#include <bachelor/TopicName.h>

void PlatformRcvBool::Callback(const std_msgs::Bool& msg)
{
   m_Message.info = msg.data;
   PlatformRcv::notifyObservers();
}

PlatformRcvBool::PlatformRcvBool(const Topic topic)
{
    m_Message.topic = topic;
    ros::NodeHandle node;
    PlatformRcv::m_Sub = node.subscribe(TopicName[topic], 1, &PlatformRcvBool::Callback, this);
}

const IMessage* PlatformRcvBool::getMessage(void) const
{
    return &m_Message;
}