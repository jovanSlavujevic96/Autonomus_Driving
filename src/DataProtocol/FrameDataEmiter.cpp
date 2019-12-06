#include <bachelor/DataProtocol/FrameDataEmiter.hpp>

#include <ros/ros.h>

class FrameDataEmiter::ImplFrameEmiter
{
    image_transport::Publisher Publisher;
public:
    ImplFrameEmiter(const std::string _topicName);
    ~ImplFrameEmiter() = default;

    void Publish(const sensor_msgs::Image &_frame);
};

FrameDataEmiter::ImplFrameEmiter::ImplFrameEmiter(const std::string _topicName) //"/video_det"
{
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    Publisher = it.advertise(_topicName,1);
}

void FrameDataEmiter::ImplFrameEmiter::Publish(const sensor_msgs::Image &_frame)
{
	Publisher.publish(_frame);
}

FrameDataEmiter::FrameDataEmiter(const std::string _topicName) : m_PimplFrameEmiter{std::make_unique<FrameDataEmiter::ImplFrameEmiter>(_topicName) }
{

}

FrameDataEmiter::~FrameDataEmiter() = default;

void FrameDataEmiter::Publish(const sensor_msgs::Image &_frame)
{
    m_PimplFrameEmiter->Publish(_frame);
}