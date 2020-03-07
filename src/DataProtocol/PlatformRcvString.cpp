#include <bachelor/DataProtocol/PlatformRcvString.hpp>
#include <bachelor/TopicName.h>
#include <bachelor/IObserver.hpp>

void PlatformRcvString::Callback(const std_msgs::String& msg)
{
    m_Message.text[0] = (msg.data);
    PlatformRcv::notifyObservers();
}  

PlatformRcvString::PlatformRcvString(const Topic topic)
{
    m_Message.topic = topic;
    m_Message.text = std::vector<std::string>(1);
    ros::NodeHandle node;
    PlatformRcv::m_Sub = node.subscribe(TopicName[topic], 1, &PlatformRcvString::Callback, this);
}
    
const IMessage* PlatformRcvString::getMessage(void) const
{
    return &m_Message;
}