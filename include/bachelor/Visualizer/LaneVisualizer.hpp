#ifndef BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_

#include "IVisualizer.hpp"

class LaneVisualizer : public IVisualizer
{
    std::vector<std::vector<cv::Point>> m_Lines;
    cv::Scalar m_LineColor, m_TextColor;
    std::string m_Direction;
    bool m_CoordinatesReceived;

public:
    LaneVisualizer();
    virtual ~LaneVisualizer() = default;

    void update(const bachelor::Coordinates& msg, Topics subjTopic) override;
    bool doStuff(void) override;

    void draw(cv::Mat& frame) override;
    VisualizerType getVisualizerType(void) override;
};

#endif //BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_