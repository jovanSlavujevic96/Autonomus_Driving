#include <bachelor/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <bachelor/Frame.h>

#define Red cv::Scalar(0,0,255)
#define Yellow cv::Scalar(0, 255, 255)
#define Lilac cv::Scalar(255,0,255)

bool Display::calculateRecievement(void)
{
    for(auto topic : m_TopicsInUse)
    {
        if(!m_Recievement[topic])
        {
            return false;
        }
    }
    return true;
}

void Display::resetVariables(void)
{
    for(auto topic : m_TopicsInUse)
    {
        m_Points[topic].clear();
        m_Recievement[topic] = false;
    }
    m_FrameRecevied = false;
}

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

void Display::addVisualizer(IVisualizer* visualizer, Topics subjTopic)
{
    for(auto topics : m_TopicsInUse)
    {
        if(topics == subjTopic)
        {
            std::cout << "You cannot add this Visualizer. Visualizer is already in use!\n";
            return;
        }
    }
    m_Visualizers[subjTopic] = visualizer;
    m_Recievement[subjTopic] = false;   //waits to become true
    m_TopicsInUse.push_back(subjTopic);   
}

bool Display::doStuff(void) 
{
    {   
        //msg instance dissapears out of scope
        std_msgs::Bool msg;
        msg.data = true;
        m_ToWatchdog->Publish(msg);
    }
    if(m_FrameRecevied && Display::calculateRecievement() )
    {
        Frame frame;
        frame.MatFrame = &m_Frame;
        
        for(auto topic : m_TopicsInUse)
        {
            for(int i=0; i<m_Points[topic].size(); ++i)
            {
                frame.Dots.push_back({m_Points[topic][i*2], m_Points[topic][(i*2)+1] });
            }
            m_Visualizers[topic]->draw(&frame);
            frame.Dots.clear();
        }
        try
        { 
            cv::imshow("Visualization: ", m_Frame);
        }
        catch(cv::Exception &error)
        {
            return false;
        }
        Display::resetVariables();
    }
    if(!Display::pauseCheck() )	//if exit button (<esc> or 'q') is pressed exit the program
	{
        return false;
    }
    return true;
}

void Display::update(const sensor_msgs::Image& msg, Topics subjTopic) 
{
    if(!m_FrameRecevied)
    {
        m_FrameRecevied = true;
        m_Frame = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
    }
}

void Display::update(const bachelor::Coordinates& msg, Topics subjTopic)
{
    m_Recievement[subjTopic] = true;
    for(int i=0; i<msg.size; ++i)
    {
        m_Points[subjTopic].push_back(cv::Point(msg.X1[i], msg.Y1[i]) );
        m_Points[subjTopic].push_back(cv::Point(msg.X2[i], msg.Y2[i]) );
        //std::cout << msg.X1 << ' ' << msg.Y1 << ' ' << msg.X2 << ' ' << msg.Y2 << std::endl;
    }
}
