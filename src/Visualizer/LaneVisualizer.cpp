#include <bachelor/Visualizer/LaneVisualizer.hpp>

#define NumOfCoord 4
#define Red cv::Scalar(0,0,255)

LaneVisualizer::LaneVisualizer() : m_Lines{std::vector<std::vector<cv::Point>>(NumOfCoord)}
{
    this->m_LineColor = cv::Scalar(0, 255, 255); //Yellow
    this->m_TextColor = cv::Scalar(0, 255, 255);
    this->m_CoordinatesReceived = false;
}

void LaneVisualizer::update(const bachelor::Coordinates &_msg, Topics _subjTopic)
{
    m_CoordinatesReceived = true;
    for(int i=0; i<NumOfCoord; ++i)
    {
        std::vector<cv::Point> pts(2);
        pts[0] = cv::Point(_msg.X1[i], _msg.Y1[i]);
        pts[1] = cv::Point(_msg.X2_Width[i], _msg.Y2_Height[i]);
        m_Lines[i] = pts;
    }
}

bool LaneVisualizer::doStuff(void)
{
    //not in use
    return true;
}

void LaneVisualizer::draw(cv::Mat &frame)
{
    if(frame.empty() || !m_CoordinatesReceived)
    {
        return;
    }

    bool rightLine=false, leftLine=false;
    
    //if there are lines draw them
    if( (m_Lines[0][0] != cv::Point(0,0)) && (m_Lines[0][1] != cv::Point(0,0)) )    //if both dots are zeros, there are no line
    {
        rightLine = true;
        cv::line(frame, m_Lines[0][0], m_Lines[0][1], m_LineColor, 5, CV_AA);
    }
    if( (m_Lines[1][0] != cv::Point(0,0)) && (m_Lines[1][1] != cv::Point(0,0)) )
    {
        leftLine = true;
        cv::line(frame, m_Lines[1][0], m_Lines[1][1], m_LineColor, 5, CV_AA);
    }
    if(rightLine && leftLine)
    {
        cv::line(frame, m_Lines[2][0], m_Lines[2][1], m_LineColor, 3, CV_AA); //measuredDot (Line)
        cv::line(frame, m_Lines[3][0], m_Lines[3][1], Red, 3, CV_AA); //referent Dot (line) //red color

        cv::Mat tmp = frame.clone();
        std::vector<cv::Point> poly_points(4);
        poly_points[0] = (m_Lines[0][0]);
        poly_points[1] = (m_Lines[0][1]);
        poly_points[2] = (m_Lines[1][1]);
        poly_points[3] = (m_Lines[1][0]);
        cv::fillConvexPoly(tmp, poly_points, Red, CV_AA, 0);
        cv::addWeighted(tmp, 0.3f, frame, 1.0f - 0.3f, 0, frame);
    }
}

VisualizerType LaneVisualizer::getVisualizerType(void)
{
    return LaneVizType;
}