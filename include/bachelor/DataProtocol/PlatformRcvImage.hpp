#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCVIMAGE_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCVIMAGE_HPP_

#include "PlatformRcv.hpp"
#include <sensor_msgs/Image.h>
#include <bachelor/Message/ImageMessage.hpp>
#include <image_transport/image_transport.h>

class PlatformRcvImage : public PlatformRcv
{
    image_transport::Subscriber m_Sub;
    ImageMessage m_Message;
    void Callback(const sensor_msgs::ImageConstPtr& msg);

public:
    PlatformRcvImage(const Topic topic);
    virtual ~PlatformRcvImage() = default;

    const IMessage* getMessage(void) const override;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCVIMAGE_HPP_