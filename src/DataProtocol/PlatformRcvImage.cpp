#include <bachelor/DataProtocol/PlatformRcvImage.hpp>
#include <bachelor/IObserver.hpp>
#include <bachelor/TopicName.h>
#include <cv_bridge/cv_bridge.h>

void PlatformRcvImage::Callback(const sensor_msgs::ImageConstPtr& msg)
{
    m_Message.image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    PlatformRcv::notifyObservers();
}

PlatformRcvImage::PlatformRcvImage(const Topic topic)
{
    m_Message.topic = topic;
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    PlatformRcvImage::m_Sub = it.subscribe(TopicName[topic], 1, &PlatformRcvImage::Callback, this);
}

const IMessage* PlatformRcvImage::getMessage(void) const
{
    return &m_Message;
}

