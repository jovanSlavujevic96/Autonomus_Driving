#ifndef BACHELOR_VISUALIZER_IVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_IVISUALIZER_HPP_

#include <bachelor/Frame.h>

enum VisualizerType{LaneVizType, StopVizType, LimitVizType, LogVizType};

class IVisualizer
{
public:
    explicit IVisualizer() = default;
    virtual ~IVisualizer() = default;

    virtual bool draw(Frame& frame) = 0;
    virtual VisualizerType getVisualizerType(void) const = 0;
};

#endif //BACHELOR_VISUALIZER_IVISUALIZER_HPP_