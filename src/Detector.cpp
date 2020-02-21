#include <bachelor/Detector.hpp>
#include <bachelor/DataSender/DataSender.hpp>

#include <iostream>

Detector::Detector(std::unique_ptr<IImageProcessor> procType) :	
	m_DataEmiterWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(procType->getWatchdogTopic() ) },
	m_CoordSender{std::make_unique<DataSender<bachelor::Coordinates>>(procType->getCoordinateTopic() ) },
	m_ImgProc{std::move(procType) }
{

}

void Detector::update(const sensor_msgs::Image &_msg, Topics _subjTopic)
{
	if(_subjTopic == RawFrame)
	{
		m_ImgProc->setFrame(_msg);
		auto Vecs = m_ImgProc->getCoordinates();
		bachelor::Coordinates msg;
		msg.size = Vecs.size();
		for(int i=0; i<Vecs.size(); i++)
		{
			auto tmpVec = Vecs[i];
			msg.X1.push_back(tmpVec[0]);
			msg.Y1.push_back(tmpVec[1]);
			msg.X2.push_back(tmpVec[2]);
			msg.Y2.push_back(tmpVec[3]);
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