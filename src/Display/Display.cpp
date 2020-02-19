#include <bachelor/Display/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <numeric>

#define NumOfDetectors (sizeof(Display::m_DisplayIt)/sizeof(*Display::m_DisplayIt))
#define Red cv::Scalar(0,0,255)
#define Yellow cv::Scalar(0, 255, 255)
#define Lilac cv::Scalar(255,0,255)

bool Display::pauseCheck(void)
{
    auto btn = cv::waitKey(1);
    if(btn == 'p' || btn == 32) //<space> asci val -> 32
    {
        m_Ignore = false;
        m_Pause = !m_Pause;
        return true;
    }
    else if(btn == 'q' || btn == 27) //<esc> asci val -> 27
    {
        return false;
    }
    m_Ignore = true;
    return true;
}

Display::Display() : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(PauseOrPlay)},
    m_ToWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_Visual)}
{
    this->m_Ignore = false;
    this->m_Pause = false;
    this->m_FrameRecevied = false;
}

void Display::addVisualizer(IVisualizer* _visualizer)
{
    m_Visualizers.push_back(_visualizer);
}

bool Display::doStuff(void) 
{
    {   
        //msg instance dissapears out of scope
        std_msgs::Bool msg;
        msg.data = true;
        m_ToWatchdog->Publish(msg);
    }
    if(!m_Ignore)	//if (<space> or 'p') pressed -> m_Ignore = false -> send pause or start
    {
        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseSender->Publish(msg); //send pause or start to publisher (VideoPlayerNode)
    }
    if(m_FrameRecevied)
    {
        try
        {
            cv::imshow("Visualization: ", m_Frame);
        }
        catch(cv::Exception &error)
        {
            return false;
        }
    }
    if(!Display::pauseCheck() )	//if exit button (<esc> or 'q') is pressed exit the program
	{
        return false;
    }
    return true;
}

void Display::update(const sensor_msgs::Image &_msg, Topics _subjTopic) 
{
    if(_subjTopic == RawFrame)    
    {
        m_Frame = cv_bridge::toCvCopy(_msg, "bgr8")->image.clone();
        m_FrameRecevied = true;
    }
}

void Display::update(const bachelor::Coordinates &_msg, Topics _subjTopic) 
{
    if(!m_FrameRecevied)
    {
        return;
    }

    switch(_subjTopic)
    {
        case Coord_LaneDet:
        {
            for(int i=0; i<m_Visualizers.size(); ++i)
            {
                if(m_Visualizers[i]->getVisualizerType() == Lane)
                {
                    m_Visualizers[i]->setCoordinates(_msg);
                    //m_Visualizers[i]->setText(DIRECTION); //in progress
                    m_Visualizers[i]->drawMe(m_Frame);
                    
                    break; //get out from for loop
                }
            }
            break;
        }
        case Coord_StopDet:
        {
            for(int i=0; i<m_Visualizers.size(); ++i)
            {
                if(m_Visualizers[i]->getVisualizerType() == Stop)
                {
                    m_Visualizers[i]->setCoordinates(_msg);
                    m_Visualizers[i]->drawMe(m_Frame);
                    
                    break; //get out from for loop
                }
            }
            break;
        }
        case Coord_LimDet:
        {
            for(int i=0; i<m_Visualizers.size(); ++i)
            {
                if(m_Visualizers[i]->getVisualizerType() == Limit)
                {
                    m_Visualizers[i]->setCoordinates(_msg);
                    m_Visualizers[i]->drawMe(m_Frame);
                    
                    break; //get out from for loop
                }
            }
            break;
        }
    }
}
