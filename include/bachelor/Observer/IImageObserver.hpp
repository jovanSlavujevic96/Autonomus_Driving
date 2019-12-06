#ifndef BACHELOR_OBSERVER_IIMAGEOBSERVER_HPP_
#define BACHELOR_OBSERVER_IIMAGEOBSERVER_HPP_

#include <image_transport/image_transport.h>
#include <bachelor/Topics.h>

//ImageObserver interface
class IImageObserver
{
public:
	explicit IImageObserver() = default;
	virtual ~IImageObserver() = default;

	virtual void update(sensor_msgs::Image &_frame, Topics _subjTopic) = 0;	//observer method
};

#endif //BACHELOR_OBSERVER_IIMAGEOBSERVER_HPP_