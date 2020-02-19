#include <bachelor/Visualizer/ObjectVisualizer.hpp>

ObjectVisualizer::ObjectVisualizer(VisualizerType type) : m_VisualizerType{type}
{
    if(type == StopVizType)
    {
        this->m_SignName = "STOP";
        this->m_TextColor = cv::Scalar(0,0,255); //Red;
        this->m_RectColor = cv::Scalar(255,0,255); //Lilac;
    }
}

bool ObjectVisualizer::doStuff(void)
{
    //not in use
    return true;
}

void ObjectVisualizer::update(const bachelor::Coordinates &_msg, Topics _subjTopic)
{
    for(int i=0; i<4; ++i)
    {
        m_Rects.push_back(cv::Rect(_msg.X1[i], _msg.Y1[i], _msg.X2_Width[i], _msg.Y2_Height[i]) );
    }
}

void ObjectVisualizer::draw(cv::Mat &frame)
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

