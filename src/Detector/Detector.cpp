#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataSender/DataSender.hpp>

Detector::Detector(std::unique_ptr<IImageProcessor> procType) :	
	m_DataEmiterWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(procType->getWatchdogTopic() ) },
	m_CoordSender{std::make_unique<DataSender<bachelor::Coordinates>>(procType->getCoordinateTopic() ) },
	m_ToECU{std::make_unique<DataSender<std_msgs::String>>(procType->getECUTopic() ) },
	m_ImgProc{std::move(procType) }
{

}

void Detector::update(const sensor_msgs::Image& msg, const Topic subjTopic)
{
	if(subjTopic == RawFrame)
	{
		m_ImgProc->setFrame(msg);

		std_msgs::String msg2;
		msg2.data = m_ImgProc->getResult();
		m_ToECU->Publish(msg2);

		auto Vecs = m_ImgProc->getCoordinates();
		bachelor::Coordinates msg3;
		msg3.size = Vecs.size();
		for(int i=0; i<Vecs.size(); i++)
		{
			auto tmpVec = Vecs[i];
			msg3.X1.push_back(tmpVec[0]);
			msg3.Y1.push_back(tmpVec[1]);
			msg3.X2.push_back(tmpVec[2]);
			msg3.Y2.push_back(tmpVec[3]);
		}
		m_CoordSender->Publish(msg3);
	}
}

bool Detector::doStuff(void)
{
	std_msgs::Bool msg;
	msg.data = true;
	m_DataEmiterWatchdog->Publish(msg);
	
	return true;
}