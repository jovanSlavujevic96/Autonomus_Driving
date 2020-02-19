#ifndef BACHELOR_DETECTOR_HPP_
#define BACHELOR_DETECTOR_HPP_

#include <memory>

#include <bachelor/IObserver.hpp> 
#include <bachelor/DataSender/IDataSender.hpp>

#include <bachelor/ImageProcessor/IImageProcessor.hpp>

#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <bachelor/Coordinates.h>

class Detector : public IObserver<sensor_msgs::Image>
{
private:
	std::unique_ptr<IDataSender<std_msgs::Bool>> m_DataEmiterWatchdog;
	std::unique_ptr<IDataSender<bachelor::Coordinates>> m_CoordSender;
	std::unique_ptr<IImageProcessor> m_ImgProcDI;

public:
	Detector(std::unique_ptr<IImageProcessor> procType, Topics ImHereTopic, Topics CoordTopic);
	virtual ~Detector() = default;
	
	void update(const sensor_msgs::Image& msg, Topics subjTopic) override;	//observer method
	bool doStuff(void) override;
};

#endif //BACHELOR_DETECTOR_HPP_
