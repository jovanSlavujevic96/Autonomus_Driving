#include <bachelor/CameraSimulator.hpp>
#include <bachelor/DataSender/DataSender.hpp>
#include <cv_bridge/cv_bridge.h>

CameraSimulator::CameraSimulator() :
	m_FrameEmiter{std::make_unique<DataSender<sensor_msgs::Image>>(RawFrame) }, //fromVIDEOPtoOBJDET
	m_WatchdogEmiter{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_CamSim) },
	m_NodeMSG{false}
{

}

void CameraSimulator::setVideo(cv::VideoCapture& video)
{
	m_Video = video;
}

void CameraSimulator::checkMsgs(void)
{
	std_msgs::Bool msg;
	msg.data = true;
	m_WatchdogEmiter->Publish(msg);
	m_PauseVideo = m_NodeMSG;
}

void CameraSimulator::update(const std_msgs::Bool& msg, const Topic subjTopic)
{
	if(subjTopic == PauseOrPlay)
	{
		m_NodeMSG = msg.data;
	}
}

bool CameraSimulator::doStuff(void)
{
	CameraSimulator::checkMsgs();
	if(!m_PauseVideo)
	{
		cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );

		cv::Mat frame;
		m_Video >> frame;
		if(frame.empty() )
		{
			return false;
		}
		cv_ptr->encoding = "bgr8";
		cv_ptr->image = frame;

		sensor_msgs::Image img1;
		cv_ptr->toImageMsg(img1);
		m_FrameEmiter->Publish(img1);
	}
	return true;
}