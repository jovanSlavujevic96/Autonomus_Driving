#ifndef BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_
#define BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_

#include <memory>

#include <bachelor/Observer/IObserver.hpp> 
#include <bachelor/DataProtocol/Template/IDataSender.hpp>

#include "IImageProcessor.hpp"

#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

//observer
template class IObserver<sensor_msgs::Image>;
class ObjectDetector : public IObserver<sensor_msgs::Image>
{
private:
	std::unique_ptr<IDataSender<bool, std_msgs::Bool> > m_DataEmiterWatchdog, m_DataEmiterVideoPlayer;
	std::unique_ptr<IDataSender<sensor_msgs::Image, sensor_msgs::Image> > m_FrameEmiterDisplay;
	std::vector<IImageProcessor *> m_ImgProcVec;

public:
	ObjectDetector();
	virtual ~ObjectDetector() = default;

	void addImageProcessor(IImageProcessor *_processor);
	
	virtual void update(sensor_msgs::Image &_frame, Topics _subjTopic) override;	//observer method
	virtual bool doStuff(void) override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_
