#include <bachelor/Visualizer/ObjectVisualizer.hpp>

ObjectVisualizer::ObjectVisualizer(VisualizerType type) : m_VisualizerType{type}
{
    if(type == Stop)
    {
        this->m_SignName = "STOP";
    }
}

void ObjectVisualizer::setCoordinates(const bachelor::Coordinates &coordinates) 
{
    for(int i=0; i<4; ++i)
    {
        m_Rects.push_back(cv::Rect(coordinates.X1[i], coordinates.Y1[i], coordinates.X2_Width[i], coordinates.Y2_Height[i]) );
    }
}

void ObjectVisualizer::setColor(const cv::Scalar &color)
{
    m_RectColor = color;
}

void ObjectVisualizer::setText(const std::string text, const cv::Scalar &color)
{
    m_SignName = text;
    m_TextColor = color;
}

void ObjectVisualizer::drawMe(cv::Mat &frame)
{
    int size=0;
    for(int i=0; i<m_Rects.size(); ++i)
    {
        if(m_Rects[i] == cv::Rect(0,0,0,0) )
        {
            if(i==0)
            {
                return;    //there are no rects and that's ok
            }
            size = i;
            break;
        }    
    }

    auto helpImage = frame.clone();
	for(int i=0; i<size; ++i)
    {
        cv::rectangle(frame, m_Rects[i], m_RectColor, -1);
    }
	cv::addWeighted(helpImage, 0.8f, frame, 0.2f, 0, frame);
	for(int i = 0 ; i <size; ++i) 
    {
        cv::rectangle(frame, m_Rects[i], m_RectColor, 3);
        cv::putText(frame, m_SignName, cv::Point(m_Rects[i].x+1, (m_Rects[i].width+m_Rects[i].y+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, m_TextColor, 1);
    }
    m_Rects.clear();    //reset vector
}

VisualizerType ObjectVisualizer::getVisualizerType(void)
{
    return m_VisualizerType;
}

