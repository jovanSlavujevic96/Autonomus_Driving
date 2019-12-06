#ifndef BACHELOR_DATAPROTOCOL_IFRAMEDATAEMITER_HPP_
#define BACHELOR_DATAPROTOCOL_IFRAMEDATAEMITER_HPP_

#include <image_transport/image_transport.h>

//Frame Data Emiter interface
class IFrameDataEmiter
{
public:
    explicit IFrameDataEmiter() = default;
    virtual ~IFrameDataEmiter() = default;    

    virtual void Publish(const sensor_msgs::Image &_frame) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IFRAMEDATAEMITER_HPP_