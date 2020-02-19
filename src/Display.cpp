#include <bachelor/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <numeric>

#define Red cv::Scalar(0,0,255)
#define Yellow cv::Scalar(0, 255, 255)
#define Lilac cv::Scalar(255,0,255)

bool Display::pauseCheck(void)
{
    auto btn = cv::waitKey(1);
    if(btn == 'p' || btn == 32) //<space> asci val -> 32
    {
        m_Pause = !m_Pause; //change state

        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseSender->Publish(msg); //publish message to CameraSimulator_Node
        
        return true;
    }
    else if(btn == 'q' || btn == 27) //<esc> asci val -> 27
    {
        return false;
    }
    return true;
}

Display::Display() : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(PauseOrPlay)},
    m_ToWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_Visual)}
{
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
    m_FrameRecevied = true;
    m_Frame = cv_bridge::toCvCopy(_msg, "bgr8")->image.clone();
    for(int i=0; i<m_Visualizers.size(); ++i)
    {
        m_Visualizers[i]->draw(m_Frame);
    }
}
