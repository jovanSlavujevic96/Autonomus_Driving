#include <bachelor/VideoPlayerNode/VideoPlayer.hpp>
#include <bachelor/DataProtocol/Template/DataSender.hpp>
#include <cv_bridge/cv_bridge.h>

VideoPlayer::VideoPlayer() :
	m_FrameEmiter{std::make_unique<DataSender<sensor_msgs::Image,sensor_msgs::Image> >(fromVIDEOPtoOBJDET) },
	m_TimerStartsEmiter{std::make_unique<DataSender<bool,std_msgs::Bool> >(fromVIDEOPtoTIMER) }, 
	m_WatchdogEmiter{std::make_unique<DataSender<bool,std_msgs::Bool> >(fromVIDEOPtoWDOG) },
	m_NodeMSG{false}
{
	system("clear");
}

VideoPlayer::~VideoPlayer()
{
	system("clear");
}

void VideoPlayer::setVideo(cv::VideoCapture &_video)
{
	m_Video = _video;
}

void VideoPlayer::checkMsgs(void)
{
	m_WatchdogEmiter->Publish(true);
	if(!m_IgnoreDetection)
	{
		if(m_NodeMSG[0])
		{
			m_TimerStartsEmiter->Publish(true);
			m_NodeMSG[0] = false;
			m_IgnoreDetection = true;
			m_PauseVideo = true;
		}
		else
		{
			m_PauseVideo = m_NodeMSG[1];
		}
	}
	else
	{
		if(m_PauseVideo && m_NodeMSG[2])
		{
			m_TimerStartsEmiter->Publish(true);
			m_NodeMSG[2] = false;
			m_PauseVideo = false;
		}
		else if(!m_PauseVideo && m_NodeMSG[2])
		{
			m_NodeMSG[2] = false;
			m_IgnoreDetection = false;
		}
	}
}

void VideoPlayer::update(bool &_data, Topics _subjTopic)
{
	if(_subjTopic == fromOBJDETtoVIDEOP)
		m_NodeMSG[0] = _data;
	else if(_subjTopic == fromDISPtoVIDEOP)
		m_NodeMSG[1] = _data;
	else if(_subjTopic == fromTIMERtoVIDEOP)
		m_NodeMSG[2] = _data;
}

bool VideoPlayer::doStuff(void)
{
	VideoPlayer::checkMsgs();
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
		return true;
	}
	else 
		return true;
}