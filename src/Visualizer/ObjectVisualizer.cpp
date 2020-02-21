#include <bachelor/Visualizer/ObjectVisualizer.hpp>

ObjectVisualizer::ObjectVisualizer(VisualizerType type) : 
    m_VisualizerType{type}
{
    if(type == StopVizType)
    {
        this->m_SignName = "STOP";
        this->m_TextColor = cv::Scalar(0,0,255); //Red;
        this->m_RectColor = cv::Scalar(255,0,255); //Lilac;
    }
    else if(type == LimitVizType)
    {
        this->m_SignName = "LIMIT";
        this->m_TextColor = cv::Scalar(255,255,0); //Light Blue;
        this->m_RectColor = cv::Scalar(255,0,0); //Blue;
    }
}

void ObjectVisualizer::draw(Frame* frame)
{
    for(int i=0; i<(frame->Dots.size() ); ++i)
    {
        m_Rects.push_back(cv::Rect(frame->Dots[i][0], frame->Dots[i][1] ));
    }

    int size=0;
    for(int i=0; i<m_Rects.size(); ++i)
    {
        if(m_Rects[i] == cv::Rect(0,0,0,0) )
        {
            if(i==0)
            {
                m_Rects.clear();    //reset vector
                return;    //there are no rects and that's ok
            }
            size = i;
            break;
        }    
    }
    auto helpImage = (*frame->MatFrame).clone();
	for(int i=0; i<size; ++i)
    {
        cv::rectangle(*frame->MatFrame, m_Rects[i], m_RectColor, -1);
    }
	cv::addWeighted(helpImage, 0.8f, *frame->MatFrame, 0.2f, 0, *frame->MatFrame);
	for(int i = 0 ; i <size; ++i) 
    {
        cv::rectangle(*frame->MatFrame, m_Rects[i], m_RectColor, 3);
        cv::putText(*frame->MatFrame, m_SignName, cv::Point(m_Rects[i].x+1, (m_Rects[i].width+m_Rects[i].y+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, m_TextColor, 1);
    }
    m_Rects.clear();    //reset vector
}

VisualizerType ObjectVisualizer::getVisualizerType(void)
{
    return m_VisualizerType;
}

