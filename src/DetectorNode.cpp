#include <bachelor/DetectorNode.hpp>
#include <ros/ros.h>

void DetectorNode::init(void)
{
    m_FrameRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(RawFrame);
    m_FrameRcv->registerObserver(m_Detector.get() );
}

DetectorNode::DetectorNode(std::unique_ptr<IImageProcessor> processor) :
    m_Detector{std::make_unique<Detector>(std::move(processor) )}
{
    DetectorNode::init();
}

DetectorNode::DetectorNode(std::unique_ptr<Detector> detector) :
    m_Detector{std::move(detector)}
{
    DetectorNode::init();
}

void DetectorNode::runProgram(void)
{
    while(ros::ok() && m_Detector->doStuff() )
	{
		ros::spinOnce(); //spin for new event on topics
	}
}   