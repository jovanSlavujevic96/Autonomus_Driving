#ifndef BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_

#include "IVisualizer.hpp"

class LogVisualizer :
    public IVisualizer
{
    int m_FrameWidth, m_FrameHeight;
    cv::Rect m_WhiteRegion;
    cv::Point m_MovementPoint, m_LimitPoint;

    void assignParameters(const cv::Mat &frame);
public:
    LogVisualizer();
    virtual ~LogVisualizer() = default;

    bool draw(Frame& frame) override;
    VisualizerType getVisualizerType(void) const override;
};

#endif //BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_