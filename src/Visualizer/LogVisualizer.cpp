#include <bachelor/Visualizer/LogVisualizer.hpp>

#define white cv::Scalar(255,255,255)

void LogVisualizer::assignParameters(const cv::Mat &frame)
{
    if(m_FrameHeight && m_FrameWidth)
    {
        return;
    }
    m_FrameWidth = frame.size().width;
    m_FrameHeight = frame.size().height; 
}

LogVisualizer::LogVisualizer()
{
    this->m_FrameWidth = 0;
    this->m_FrameWidth = 0;
}

void LogVisualizer::draw(Frame* frame)
{
    LogVisualizer::assignParameters(*frame->MatFrame);

    cv::Rect rectangle;
    {
        auto pt1 = cv::Point(0, 0), pt2 = cv::Point(m_FrameWidth, std::round(m_FrameHeight/5));
        rectangle = cv::Rect(pt1, pt2);
    }
    cv::rectangle(*frame->MatFrame, rectangle, white, -1);
}

VisualizerType LogVisualizer::getVisualizerType(void)
{
    return LogVizType;
}