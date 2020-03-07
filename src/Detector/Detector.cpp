#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataProtocol/Sender.hpp>
#include <bachelor/Message/IMessage.hpp>
#include <bachelor/Message/BoolMessage.hpp>
#include <bachelor/Message/ImageMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>
#include <bachelor/DataProtocol/IPlatformRcv.hpp>

Detector::Detector(std::unique_ptr<IImageProcessor> procType) :	
	m_DataEmiterWatchdog{std::make_unique<Sender<bool>>(procType->getWatchdogTopic() ) },
	m_CoordSender{std::make_unique<Sender<std::vector<std::vector<cv::Point>>>>(procType->getCoordinateTopic() ) },
	m_ToECU{std::make_unique<Sender<std::string>>(procType->getECUTopic() ) },
	m_ImgProc{std::move(procType) }
{

}

void Detector::update(const IPlatformRcv* receiver)
{
	auto msg1 = static_cast<const ImageMessage*>(receiver->getMessage());
	if(msg1->topic == RawFrame)
	{
		m_ImgProc->setFrame(msg1);
		m_ToECU->Publish(m_ImgProc->getDetectionMessage());
		m_CoordSender->Publish(m_ImgProc->getCoordinateMessage());
	}
}

bool Detector::doStuff(void)
{
	BoolMessage msg;
	msg.info = true;
	m_DataEmiterWatchdog->Publish(&msg);
	
	return true;
}