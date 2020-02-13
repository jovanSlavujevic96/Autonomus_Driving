#ifndef BACHELOR_DETECTOR_DETECTOR_HPP_
#define BACHELOR_DETECTOR_DETECTOR_HPP_

#include <memory>

#include <bachelor/IObserver.hpp> 
#include <bachelor/DataSender/IDataSender.hpp>

#include <bachelor/ImageProcessor/IImageProcessor.hpp>

#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <bachelor/Coordinates.h>

//observer
template class IObserver<sensor_msgs::Image>;
class Detector : public IObserver<sensor_msgs::Image>
{
private:
	std::unique_ptr<IDataSender<std_msgs::Bool>> m_DataEmiterWatchdog;
	std::unique_ptr<IDataSender<bachelor::Coordinates>> m_CoordSender;
	//std::unique_ptr<IDataSender<sensor_msgs::Image>> m_FrameEmiterDisplay;
	std::unique_ptr<IImageProcessor> m_ImgProcDI;

public:
	Detector(std::unique_ptr<IImageProcessor> _procType);
	virtual ~Detector() = default;
	
	void update(sensor_msgs::Image &_frame, Topics _subjTopic) override;	//observer method
	bool doStuff(void) override;
};

#endif //BACHELOR_DETECTOR_DETECTOR_HPP_
