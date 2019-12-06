#include <bachelor/DataProtocol/BoolDataEmiter.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class BoolDataEmiter::ImplDataEmiter
{
	ros::Publisher Publisher;
public:
	ImplDataEmiter(const std::string _topicName);
	~ImplDataEmiter() = default;

	void Publish(const bool data);
};

BoolDataEmiter::ImplDataEmiter::ImplDataEmiter(const std::string _topicName)
{
	ros::NodeHandle nh;
	Publisher = nh.advertise<std_msgs::Bool>(_topicName, 1);
}

BoolDataEmiter::BoolDataEmiter(const std::string _topicName) : m_PimplDataEmiter{std::make_unique<ImplDataEmiter>(_topicName) }
{

}

BoolDataEmiter::~BoolDataEmiter() = default;

void BoolDataEmiter::ImplDataEmiter::Publish(const bool data)
{
	std_msgs::Bool msg;
	msg.data = data;
	Publisher.publish(msg);
}

void BoolDataEmiter::Publish(const bool data)
{
	m_PimplDataEmiter->Publish(data);
}