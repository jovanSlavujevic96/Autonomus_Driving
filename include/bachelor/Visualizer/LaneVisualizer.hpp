#ifndef BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_

#include "IVisualizer.hpp"

class LaneVisualizer : 
    public IVisualizer
{
    cv::Scalar m_LineColor, m_TextColor;
    std::string m_Direction;

public:
    LaneVisualizer();
    virtual ~LaneVisualizer() = default;

    bool draw(Frame& frame) override;
    VisualizerType getVisualizerType(void) const override;
};

#endif //BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_