#include <bachelor/Detector.hpp>
#include <bachelor/DataSender/DataSender.hpp>

#include <iostream>

Detector::Detector(std::unique_ptr<IImageProcessor>_processor, Topics _ImHereTopic, Topics _CoordTopic) :	
	m_DataEmiterWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(_ImHereTopic) },
	m_CoordSender{std::make_unique<DataSender<bachelor::Coordinates>>(_CoordTopic) },
	m_ImgProcDI{std::move(_processor) }
{

}

void Detector::update(const sensor_msgs::Image &_msg, Topics _subjTopic)
{
	if(_subjTopic == RawFrame)
	{
		m_ImgProcDI->setFrame(_msg);
		auto Vecs = m_ImgProcDI->getCoordinates();
		bachelor::Coordinates msg;
		for(int i=0; i<4; i++)
		{
			auto tmpVec = Vecs[i];
			msg.X1[i] = tmpVec[0];
			msg.Y1[i] = tmpVec[1];
			msg.X2_Width[i] = tmpVec[2];
			msg.Y2_Height[i] = tmpVec[3];
		}
		m_CoordSender->Publish(msg);
	}
}

bool Detector::doStuff(void)
{
	std_msgs::Bool msg;
	msg.data = true;
	m_DataEmiterWatchdog->Publish(msg);
	
	return true;
}