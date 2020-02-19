#ifndef BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP

#include <image_transport/image_transport.h>
#include <string>
#include <vector>

class IImageProcessor
{
public:
    explicit IImageProcessor() = default;
    virtual ~IImageProcessor() = default;

    virtual void setFrame(const sensor_msgs::Image& frame) = 0;
    virtual sensor_msgs::Image getProcessedFrame(void) const = 0;
    virtual bool getDetection(void) const = 0;
    virtual std::string getResult(void) const = 0;
    virtual std::vector<std::vector<int>> getCoordinates(void) const = 0;
};

#endif //BACHELOR_IMAGEPROCESSOR_IIMAGEPROCESSOR_HPP