#include <bachelor/ObjectDetectorNode/ObjectDetector.hpp>

#include <bachelor/DataProtocol/DataSender.hpp>

#include <bachelor/ObjectDetectorNode/StopSignProcessor.hpp>
#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>

#include <iostream>

ObjectDetector::ObjectDetector() :	
	m_DataEmiterWatchdog{std::make_unique<DataSender<bool, std_msgs::Bool> >(fromOBJDETtoWDOG) },
	m_DataEmiterVideoPlayer{std::make_unique<DataSender<bool, std_msgs::Bool> >(fromOBJDETtoVIDEOP) },
	m_FrameEmiterDisplay{std::make_unique<DataSender<sensor_msgs::Image, sensor_msgs::Image> >(fromOBJDETtoDISP) }
{

}

void ObjectDetector::update(sensor_msgs::Image &_frame, Topics _subjTopic)
{
	if(_subjTopic == fromVIDEOPtoOBJDET)
	{
		sensor_msgs::Image tmp = _frame;
		
		for(int i=0; i<m_ImgProcVec.size(); ++i)
		{
			m_ImgProcVec[i]->setFrame(tmp);
			tmp = m_ImgProcVec[i]->getProcessedFrame();
			if( !strcmp(m_ImgProcVec[i]->getProcessorName().c_str(),"Stop Sign Processor") )
				m_DataEmiterVideoPlayer->Publish(m_ImgProcVec[i]->getDetection() );	//to freeze the frame at few seconds
		}
		
		m_FrameEmiterDisplay->Publish(tmp);
	}
}

void ObjectDetector::addImageProcessor(IImageProcessor *_processor)
{
	const std::string InputProcType = _processor->getProcessorName();
	for(int i=0; i<m_ImgProcVec.size(); ++i)
	{
		if( !strcmp(m_ImgProcVec[i]->getProcessorName().c_str(), _processor->getProcessorName().c_str() ) )
		{
			std::cout << std::endl << "You can't add this Image Processor." << std::endl;
			std::cout << "There's is one Image Processor type of: " << InputProcType << ".\n" << std::endl;
			return;
		}
		
	}
	std::cout << "Successfully added Image Processor: " << InputProcType << ".\n";
	m_ImgProcVec.push_back(_processor);
}

bool ObjectDetector::doStuff(void)
{
	m_DataEmiterWatchdog->Publish(true);
}