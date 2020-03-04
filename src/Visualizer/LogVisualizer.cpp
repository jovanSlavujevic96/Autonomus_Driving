#include <bachelor/Visualizer/LogVisualizer.hpp>

#define white cv::Scalar(255,255,255)
#define black cv::Scalar(0,0,0)
#define font cv::FONT_HERSHEY_COMPLEX_SMALL

void LogVisualizer::assignParameters(const cv::Mat &frame)
{
    if(m_FrameHeight == frame.size().height && m_FrameWidth == frame.size().width)
    {
        return;
    }
    m_FrameWidth = frame.size().width;
    m_FrameHeight = frame.size().height;
    
    auto pt1 = cv::Point(0, std::round(m_FrameHeight*4/5)), pt2 = cv::Point(m_FrameWidth, m_FrameHeight);
    m_WhiteRegion = cv::Rect(pt1, pt2); 
    
    m_MovementPoint = cv::Point(pt1.x+15, pt1.y+40);
    m_LimitPoint = cv::Point(m_MovementPoint.x, m_MovementPoint.y+40 );
}

LogVisualizer::LogVisualizer()
{
    this->m_FrameWidth = 0;
    this->m_FrameHeight = 0;
}

bool LogVisualizer::draw(Frame& frame)
{
    if((*frame.Text).empty() || frame.MatFrame->empty() )
    {
        return false;
    }
    LogVisualizer::assignParameters(*frame.MatFrame);
    //fill white
    cv::rectangle(*frame.MatFrame, m_WhiteRegion, white, -1);
    //print
    cv::putText(*frame.MatFrame, ("Movement: " + (*frame.Text)[0]), m_MovementPoint, font, 2, black, 1, CV_AA);
    cv::putText(*frame.MatFrame, ("Limit value: " + (*frame.Text)[1]), m_LimitPoint, font, 2, black, 1, CV_AA);
    return true;
}

VisualizerType LogVisualizer::getVisualizerType(void) const
{
    return LogVizType;
}