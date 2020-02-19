#ifndef BACHELOR_VISUALIZER_IVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_IVISUALIZER_HPP_

enum VisualizerType{Lane, Stop, Limit};

#include <bachelor/Coordinates.h>
#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>

class IVisualizer
{
public:
    explicit IVisualizer() = default;
    virtual ~IVisualizer() = default;

    virtual void setCoordinates(const bachelor::Coordinates &coordinates) = 0;
    virtual void setColor(const cv::Scalar &color) = 0;
    virtual void setText(const std::string text, const cv::Scalar &color) = 0;
    virtual void drawMe(cv::Mat &frame) = 0;
    virtual VisualizerType getVisualizerType(void) = 0;
};

#endif //BACHELOR_VISUALIZER_IVISUALIZER_HPP_