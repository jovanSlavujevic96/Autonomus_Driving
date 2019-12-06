#ifndef BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_
#define BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_

#include <memory>

#include <bachelor/Observer/IImageObserver.hpp> 
#include <bachelor/DataProtocol/IBoolDataEmiter.hpp>
#include <bachelor/DataProtocol/IFrameDataEmiter.hpp>

#include <bachelor/Topics.h>

#include "IImageProcessor.hpp"

//observer
class ObjectDetector : public IImageObserver
{
private:
	std::unique_ptr<IBoolDataEmiter> m_DataEmiterWatchdog, m_DataEmiterVideoPlayer;
	std::unique_ptr<IFrameDataEmiter> m_FrameEmiterDisplay;
	std::vector<IImageProcessor *> m_ImgProcVec;

public:
	ObjectDetector();
	virtual ~ObjectDetector() = default;

	virtual void update(sensor_msgs::Image &_frame, Topics _subjTopic) override;	//observer method
	void sendDataToTopic(Topics _whichTopic, bool _data);
	void addImageProcessor(IImageProcessor *_processor);

};

#endif //BACHELOR_OBJECTDETECTORNODE_OBJECTDETECTOR_HPP_
