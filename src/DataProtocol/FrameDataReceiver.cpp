#include <bachelor/DataProtocol/FrameDataReceiver.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>

class FrameDataReceiver::ImplFrameSub
{	
	image_transport::Subscriber Subscriber;
	void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
	
	std::vector<IImageObserver *> observers;
	void notifyObservers(sensor_msgs::Image &_frame);

	Topics m_Topic;

public:
	ImplFrameSub(const std::string _topicName);
	~ImplFrameSub() = default;

	void registerObserver(IImageObserver *observer, Topics _subjTopic);
	void removeObserver(IImageObserver *observer);
};

void FrameDataReceiver::ImplFrameSub::notifyObservers(sensor_msgs::Image &_frame) 
{
	for (IImageObserver *observer : observers)  // notify all observers
	{    
		observer->update(_frame, m_Topic);
	}
}

void FrameDataReceiver::ImplFrameSub::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		sensor_msgs::Image tmp = *msg;
		FrameDataReceiver::ImplFrameSub::notifyObservers(tmp);
		/*
		m_FrameImg.header = msg->header;
        m_FrameImg.height = msg->height;
        m_FrameImg.width = msg->width;
        m_FrameImg.encoding = "bgr8"; //msg->encoding;
        m_FrameImg.is_bigendian = msg->is_bigendian;
        m_FrameImg.step = msg->step;
        m_FrameImg.data = msg->data;
		*/
	}
	
	catch(cv_bridge::Exception &error)
	{
		std::cerr << "Error receiving message" << std::endl;
	}
}

FrameDataReceiver::ImplFrameSub::ImplFrameSub(const std::string _topicName) 
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	Subscriber = it.subscribe(_topicName, 1, &FrameDataReceiver::ImplFrameSub::ImageCallback, this);
}

void FrameDataReceiver::ImplFrameSub::registerObserver(IImageObserver *observer, Topics _subjTopic)
{
	m_Topic = _subjTopic;
	observers.push_back(observer);
}

void FrameDataReceiver::ImplFrameSub::removeObserver(IImageObserver *observer) 
{
    auto iterator = std::find(observers.begin(), observers.end(), observer);
    if (iterator != observers.end()) // observer found
	{
        observers.erase(iterator); // remove the observer
    }
}

FrameDataReceiver::FrameDataReceiver(const std::string _topicName) : m_PimplFrameSub{std::make_unique<ImplFrameSub>(_topicName) }
{	

}

FrameDataReceiver::~FrameDataReceiver() = default;

void FrameDataReceiver::registerObserver(IImageObserver *observer, Topics _subjTopic)
{
	m_PimplFrameSub->registerObserver(observer, _subjTopic);
}

void FrameDataReceiver::removeObserver(IImageObserver *observer) 
{
	m_PimplFrameSub->removeObserver(observer);
}

