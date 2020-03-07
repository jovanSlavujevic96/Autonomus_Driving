#ifndef BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP

#include <bachelor/Topic.h>
#include <image_transport/image_transport.h>
#include <string>
#include <vector>
#include <bachelor/Message/ImageMessage.hpp>

class IImageProcessor
{
public:
    explicit IImageProcessor() = default;
    virtual ~IImageProcessor() = default;

    virtual void setFrame(const IMessage* frame) = 0;
    virtual Topic getWatchdogTopic(void) const = 0;
    virtual Topic getCoordinateTopic(void) const = 0;
    virtual Topic getECUTopic(void) const = 0;
    virtual const IMessage* getCoordinateMessage(void) const = 0;
    virtual const IMessage* getProcFrameMessage(void) const = 0;
    virtual const IMessage* getDetectionMessage(void) const = 0;
};

#endif //BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP