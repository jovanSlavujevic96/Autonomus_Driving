#ifndef BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_

#include "IVisualizer.hpp"

class LaneVisualizer : public IVisualizer
{
    std::vector<std::vector<cv::Point>> m_Lines;
    cv::Scalar m_LineColor, m_TextColor;
    std::string m_Direction;

public:
    LaneVisualizer();
    virtual ~LaneVisualizer() = default;

    void setCoordinates(const bachelor::Coordinates &coordinates) override;
    void setColor(const cv::Scalar &color) override;
    void setText(const std::string text, const cv::Scalar &color) override;
    void drawMe(cv::Mat &frame) override;
    VisualizerType getVisualizerType(void) override;
};

#endif //BACHELOR_VISUALIZER_LANEVISUALIZER_HPP_