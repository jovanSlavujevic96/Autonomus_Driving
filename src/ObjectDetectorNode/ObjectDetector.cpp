#include <bachelor/ObjectDetectorNode/ObjectDetector.hpp>

#include <bachelor/DataProtocol/BoolDataEmiter.hpp>
#include <bachelor/DataProtocol/FrameDataEmiter.hpp>

#include <bachelor/ObjectDetectorNode/StopSignProcessor.hpp>
#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>

#include <iostream>

ObjectDetector::ObjectDetector() :	
	//m_ImgProcessor{std::make_unique<StopSignProcessor> () },
	m_DataEmiterWatchdog{std::make_unique<BoolDataEmiter> (TopicName[fromOBJDETtoWDOG] )},
	m_DataEmiterVideoPlayer{std::make_unique<BoolDataEmiter> (TopicName[fromOBJDETtoVIDEOP] )},
	m_FrameEmiterDisplay{std::make_unique<FrameDataEmiter>(TopicName[fromOBJDETtoDISP] )}
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
			if( !strcmp(m_ImgProcVec[i]->getProcessingName().c_str(),"StopSign") )
				ObjectDetector::sendDataToTopic(fromOBJDETtoVIDEOP, m_ImgProcVec[i]->getDetection() );
		}
		
		m_FrameEmiterDisplay->Publish(tmp);
	}
}

void ObjectDetector::sendDataToTopic(Topics _whichTopic, bool _data)
{
	switch(_whichTopic)
	{
		case fromOBJDETtoVIDEOP:
			m_DataEmiterVideoPlayer->Publish(_data);
		break;

		case fromOBJDETtoWDOG:
			m_DataEmiterWatchdog->Publish(_data);
		break;
	}
}

void ObjectDetector::addImageProcessor(IImageProcessor *_processor)
{
	const char *InputProcType = _processor->getProcessingName().c_str();
	for(int i=0; i<m_ImgProcVec.size(); ++i)
	{
		if( !strcmp(m_ImgProcVec[i]->getProcessingName().c_str(), InputProcType ) )
		{
			std::cout << std::endl << "You can't add this Image Processor." << std::endl;
			std::cout << "There's is one Image Processor type of: " << InputProcType << ".\n" << std::endl;
			return;
		}
	}
	std::cout << "Successfully added Image Processor: " << InputProcType << ".\n";
	m_ImgProcVec.push_back(_processor);
}
