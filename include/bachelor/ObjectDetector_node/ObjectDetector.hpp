#ifndef BACHELOR_OBJECTDETECTOR_NODE_OBJECTDETECTOR_HPP_
#define BACHELOR_OBJECTDETECTOR_NODE_OBJECTDETECTOR_HPP_

#include <memory>
#include <bachelor/Observer/IImageObserver.hpp> 
#include <bachelor/DataProtocol/IBoolDataEmiter.hpp>
#include <bachelor/DataProtocol/IFrameDataEmiter.hpp>

#include <bachelor/Topics.h>

class ImageProcessor;

//observer
class ObjectDetector : public IImageObserver
{
private:
	std::unique_ptr<IBoolDataEmiter> m_DataEmiterWatchdog, m_DataEmiterVideoPlayer;
	std::unique_ptr<IFrameDataEmiter> m_FrameEmiterDisplay;
	std::unique_ptr<ImageProcessor>  m_ImgProcessor;
	
public:
	ObjectDetector();
	virtual ~ObjectDetector();

	virtual void update(sensor_msgs::Image &_frame, Topics _subjTopic) override;	//observer method
	void sendDataToTopic(Topics _whichTopic, bool _data);

};

#endif //BACHELOR_OBJECTDETECTOR_NODE_OBJECTDETECTOR_HPP_
