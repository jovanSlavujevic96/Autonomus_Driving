#ifndef BACHELOR_OBJECTDETECTORNODE_IIMAGEPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_IIMAGEPROCESSOR_HPP

#include <image_transport/image_transport.h>
#include <string>

class IImageProcessor
{
public:
    explicit IImageProcessor() = default;
    virtual ~IImageProcessor() = default;

    virtual void setFrame(sensor_msgs::Image &rawFrame) = 0;
    virtual sensor_msgs::Image getProcessedFrame(void) const = 0;
    virtual bool getDetection(void) const = 0;
    virtual int getValue(void) const = 0;
    virtual std::string getProcessingName(void) const = 0;
};

#endif //BACHELOR_OBJECTDETECTORNODE_IIMAGEPROCESSOR_HPP