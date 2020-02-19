#ifndef BACHELOR_VISUALIZER_IVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_IVISUALIZER_HPP_

enum VisualizerType{LaneVizType, StopVizType, LimitVizType};

#include <bachelor/Coordinates.h>
#include <bachelor/IObserver.hpp>

#include <opencv2/opencv.hpp>

class IVisualizer : public IObserver<bachelor::Coordinates>
{
public:
    explicit IVisualizer() = default;
    virtual ~IVisualizer() = default;

    virtual void draw(cv::Mat& frame) = 0;
    virtual VisualizerType getVisualizerType(void) = 0;
};

#endif //BACHELOR_VISUALIZER_IVISUALIZER_HPP_