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

bool ObjectVisualizer::draw(Frame& frame)
{
    if( (*frame.Dots).empty() || frame.MatFrame->empty() || (*frame.Dots)[0][0] == cv::Point(0,0) && (*frame.Dots)[0][1] == cv::Point(0,0) )
    {
        return false;
    }
    auto helpImage = (*frame.MatFrame).clone();
	for(int i=0; i<(*frame.Dots).size(); ++i)
    {
        cv::rectangle(*frame.MatFrame, cv::Rect((*frame.Dots)[i][0], (*frame.Dots)[i][1]), m_RectColor, -1);
    }
	cv::addWeighted(helpImage, 0.8f, *frame.MatFrame, 0.2f, 0, *frame.MatFrame);
    
	for(int i = 0 ; i<(*frame.Dots).size(); ++i) 
    {
        auto rect = cv::Rect((*frame.Dots)[i][0], (*frame.Dots)[i][1]);
        cv::rectangle(*frame.MatFrame, cv::Rect((*frame.Dots)[i][0], (*frame.Dots)[i][1]), m_RectColor, 3);
        auto pt = cv::Point( rect.x+1, rect.width+rect.y+18 );
        std::string _frameText;
        if(!(*frame.Text).empty() )
        {
            _frameText = (*frame.Text)[0];
        }
        cv::putText(*frame.MatFrame, (m_SignName+_frameText), pt, cv::FONT_HERSHEY_DUPLEX, 0.7f, m_TextColor, 1);
    }
    return true;
}

VisualizerType ObjectVisualizer::getVisualizerType(void) const
{
    return m_VisualizerType;
}

