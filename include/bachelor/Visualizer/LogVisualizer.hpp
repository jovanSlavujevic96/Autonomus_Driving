#ifndef BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_

#include "IVisualizer.hpp"

class LogVisualizer :
    public IVisualizer
{
    int m_FrameWidth, m_FrameHeight;

    void assignParameters(const cv::Mat &frame);
public:
    LogVisualizer();
    virtual ~LogVisualizer() = default;

    void draw(Frame* frame) override;
    VisualizerType getVisualizerType(void) override;
};

#endif //BACHELOR_VISUALIZER_LOGVISUALIZER_HPP_