#include <bachelor/CameraSimulator.hpp>
#include <bachelor/DataProtocol/Sender.hpp>

#include <bachelor/DataProtocol/IPlatformRcv.hpp>
#include <bachelor/Message/IMessage.hpp>
#include <bachelor/Message/BoolMessage.hpp>
#include <bachelor/Message/ImageMessage.hpp>

CameraSimulator::CameraSimulator() :
	m_FrameEmiter{std::make_unique<Sender<cv::Mat>>(RawFrame) }, //fromVIDEOPtoOBJDET
	m_WatchdogEmiter{std::make_unique<Sender<bool>>(ImHere_CamSim) }
{

}

void CameraSimulator::setVideo(cv::VideoCapture& video)
{
	m_Video = video;
}

void CameraSimulator::checkMsgs(void)
{
	BoolMessage msg;
	msg.info = true;
	m_WatchdogEmiter->Publish(&msg);
}

void CameraSimulator::update(const IPlatformRcv* receiver)
{
	auto msg =  static_cast<const BoolMessage*>(receiver->getMessage());
	if(msg->topic == PauseOrPlay)
	{
		m_PauseVideo = msg->info;
	}
}

bool CameraSimulator::doStuff(void)
{
	CameraSimulator::checkMsgs();
	if(!m_PauseVideo)
	{
		ImageMessage msg;
		m_Video >> msg.image;
		if(msg.image.empty() )
		{
			return false;
		}
		m_FrameEmiter->Publish(&msg);
	}
	return true;
}