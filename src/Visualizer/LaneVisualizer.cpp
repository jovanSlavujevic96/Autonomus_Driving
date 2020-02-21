#include <bachelor/Visualizer/LaneVisualizer.hpp>

#define Red cv::Scalar(0,0,255)

LaneVisualizer::LaneVisualizer() 
{
    this->m_LineColor = cv::Scalar(0, 255, 255); //Yellow
    this->m_TextColor = cv::Scalar(0, 255, 255);
}

void LaneVisualizer::draw(Frame* frame)
{
    bool rightLine=false, leftLine=false;
    
    //if there are lines draw them
    if( (frame->Dots[0][0] != cv::Point(0,0)) && (frame->Dots[0][1] != cv::Point(0,0)) )    //if both dots are zeros, there are no line
    {
        rightLine = true;
        cv::line(*frame->MatFrame, frame->Dots[0][0], frame->Dots[0][1], m_LineColor, 5, CV_AA);
    }
    if( (frame->Dots[1][0] != cv::Point(0,0)) && (frame->Dots[1][1] != cv::Point(0,0)) )
    {
        leftLine = true;
        cv::line(*frame->MatFrame, frame->Dots[1][0], frame->Dots[1][1], m_LineColor, 5, CV_AA);
    }
    if(rightLine && leftLine)
    {
        cv::line(*frame->MatFrame, frame->Dots[2][0], frame->Dots[2][1], m_LineColor, 3, CV_AA); //measuredDot (Line)
        cv::line(*frame->MatFrame, frame->Dots[3][0], frame->Dots[3][1], Red, 3, CV_AA); //referent Dot (line) //red color

        cv::Mat tmp = (*frame->MatFrame).clone();
        std::vector<cv::Point> poly_points(4);
        poly_points[0] = (frame->Dots[0][0]);
        poly_points[1] = (frame->Dots[0][1]);
        poly_points[2] = (frame->Dots[1][1]);
        poly_points[3] = (frame->Dots[1][0]);
        cv::fillConvexPoly(tmp, poly_points, Red, CV_AA, 0);
        cv::addWeighted(tmp, 0.3f, *frame->MatFrame, 1.0f - 0.3f, 0, *frame->MatFrame);
    }
}

VisualizerType LaneVisualizer::getVisualizerType(void)
{
    return LaneVizType;
}