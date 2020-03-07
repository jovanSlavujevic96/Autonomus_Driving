#include <bachelor/DataProtocol/PlatformRcvCoord.hpp>
#include <bachelor/TopicName.h>
#include <bachelor/IObserver.hpp>

void PlatformRcvCoord::Callback(const bachelor::Coordinates& msg)
{
    auto size = static_cast<int>(msg.size);
    m_Message.coordinates = std::vector<std::vector<cv::Point>>(size);
    for(int i=0; i<size; ++i)
    {   
        m_Message.coordinates[i] = {cv::Point(msg.X1[i], msg.Y1[i]), cv::Point(msg.X2[i], msg.Y2[i]) };
    }
    PlatformRcv::notifyObservers();
}

PlatformRcvCoord::PlatformRcvCoord(const Topic topic)
{
    ros::NodeHandle node;
    m_Message.topic = topic;
    PlatformRcv::m_Sub = node.subscribe(TopicName[topic], 1, &PlatformRcvCoord::Callback, this);
}
    
const IMessage* PlatformRcvCoord::getMessage(void) const
{
    return &m_Message;
}