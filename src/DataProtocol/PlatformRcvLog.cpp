#include <bachelor/DataProtocol/PlatformRcvLog.hpp>
#include <bachelor/TopicName.h>
#include <bachelor/IObserver.hpp>

void PlatformRcvLog::Callback(const bachelor::Log& msg)
{
    m_Message.text.clear();
    m_Message.text.push_back(msg.movement);
    m_Message.text.push_back(msg.speed_limit);
    PlatformRcv::notifyObservers();
}  

PlatformRcvLog::PlatformRcvLog(const Topic topic)
{
    m_Message.topic = topic;
    ros::NodeHandle node;
    PlatformRcv::m_Sub = node.subscribe(TopicName[topic], 1, &PlatformRcvLog::Callback, this);
}
    
const IMessage* PlatformRcvLog::getMessage(void) const
{
    return &m_Message;
}