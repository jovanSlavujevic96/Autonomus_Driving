#include <bachelor/ObjectDetector_node/ObjectDetector.hpp>
#include <bachelor/DataProtocol/BoolDataEmiter.hpp>
#include <bachelor/DataProtocol/FrameDataEmiter.hpp>
#include <bachelor/ObjectDetector_node/ImageProcessor.hpp>


ObjectDetector::ObjectDetector() :	
	m_ImgProcessor{std::make_unique<ImageProcessor> () },
	m_DataEmiterWatchdog{std::make_unique<BoolDataEmiter> (TopicName(fromOBJDETtoWDOG) )},
	m_DataEmiterVideoPlayer{std::make_unique<BoolDataEmiter> (TopicName(fromOBJDETtoVIDEOP) )},
	m_FrameEmiterDisplay{std::make_unique<FrameDataEmiter>(TopicName(fromOBJDETtoDISP) )}
{

}

ObjectDetector::~ObjectDetector()
{
	cv::destroyAllWindows();
	system("clear");
}

void ObjectDetector::update(sensor_msgs::Image &_frame, Topics _subjTopic)
{
	if(_subjTopic == fromVIDEOPtoOBJDET)
	{
		sensor_msgs::Image tmp = _frame;
		m_ImgProcessor->setFrame(tmp);
		tmp = m_ImgProcessor->getProcessedFrame();
		ObjectDetector::sendDataToTopic(fromOBJDETtoVIDEOP, m_ImgProcessor->getDetection() );
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