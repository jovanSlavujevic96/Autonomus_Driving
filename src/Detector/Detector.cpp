#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataSender/DataSender.hpp>

#include <iostream>

Detector::Detector(std::unique_ptr<IImageProcessor>_processor) :	
	m_DataEmiterWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(fromOBJDETtoWDOG) },
	m_CoordSender{std::make_unique<DataSender<bachelor::Coordinates>>(fromOBJDETtoDRAW) },
	//m_FrameEmiterDisplay{std::make_unique<DataSender<sensor_msgs::Image>>(fromOBJDETtoDISP) },
	m_ImgProcDI{std::move(_processor) }
{

}

void Detector::update(sensor_msgs::Image &_frame, Topics _subjTopic)
{
	if(_subjTopic == fromCAMtoOBJDET)
	{
		m_ImgProcDI->setFrame(_frame);
		auto tmpVec = m_ImgProcDI->getCoordinates();

		bachelor::Coordinates msg;
		for(int i=0; i<tmpVec.size(); i+=4)
		{
			msg.X1[i/4] = tmpVec[i];
			msg.Y1[i/4] = tmpVec[i+1];
			msg.X2_Width[i/4] = tmpVec[i+2];
			msg.Y2_Height[i/4] = tmpVec[i+3];
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