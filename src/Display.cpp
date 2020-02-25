#include <bachelor/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <bachelor/Frame.h>
#include <bachelor/Visualizer/LogVisualizer.hpp>

#define Red cv::Scalar(0,0,255)
#define Yellow cv::Scalar(0, 255, 255)
#define Lilac cv::Scalar(255,0,255)

bool Display::calculateRecievement(void)
{
    for(auto const& rcv : m_Recievement)
    {
        if(!rcv.second)
        {
            return false;
        }
    }
    return true;
}

void Display::resetVariables(void)
{
    for(auto& rcv : m_Recievement)
    {
        rcv.second = false;
    }
    m_FrameRecevied = false;
}

bool Display::pauseCheck(void)
{
    auto btn = cv::waitKey(1);
    if(btn == 'q' || btn == 27) //<esc> asci val -> 27
    {
        return false;
    }
    else if(btn == 'p' || btn == 32) //<space> asci val -> 32
    {
        m_Pause = !m_Pause; //change state

        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseSender->Publish(msg); //publish message to CameraSimulator_Node
    }
    return true;
}

void Display::assignStringAndPoints(std::vector<std::string>& text, std::vector<std::vector<cv::Point>>& pts, const Topic topic)
{
    text.clear();
    pts.clear();

    if(topic == LogFromECU) 
    {
        text.push_back(m_ECULog.movement);
        text.push_back(m_ECULog.speed_limit);
        return;
    }

    switch(topic)
    {
        case Coord_StopDet:
        {
            text.push_back("");
            break;
        }
        case Coord_LimDet:
        {
            text.push_back(m_ECULog.speed_limit);
            break;
        }
    }
    pts = m_Points[topic];
    m_Points[topic].clear();
}

Display::Display() : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(PauseOrPlay)},
    m_ImHere{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_Visual)}
{
    this->m_Pause = false;
    this->m_FrameRecevied = false;

    m_ECULog.speed_limit = "NaN";
    m_ECULog.movement = "NaN";
}

void Display::addVisualizer(IVisualizer* visualizer, const Topic subjTopic)
{
    for(auto const& Visualizer : m_Visualizers)
    {
        if(Visualizer.second->getVisualizerType() == visualizer->getVisualizerType() )
        {
            std::cout << "You cannot add this Visualizer. Visualizer is already in use!\n";
            return;
        }
    }
    m_Visualizers[subjTopic] = visualizer;
    m_Recievement[subjTopic] = false;   //waits to become true
}

bool Display::doStuff(void) 
{
    {   
        //msg instance dissapears out of scope
        std_msgs::Bool msg;
        msg.data = true;
        m_ImHere->Publish(msg);
    }
    if(m_FrameRecevied && Display::calculateRecievement() )
    {
        Frame frame;
        frame.MatFrame = &m_Frame;
        for(auto& visualizer : m_Visualizers)
        {
            Display::assignStringAndPoints(frame.Text, frame.Dots, visualizer.first);
            visualizer.second->draw(&frame);
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
    return Display::pauseCheck();	//if exit button (<esc> or 'q') is pressed exit the program
}

void Display::update(const sensor_msgs::Image& msg, const Topic subjTopic) 
{
    if(!m_FrameRecevied && subjTopic == RawFrame)
    {
        m_FrameRecevied = true;
        m_Frame = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
    }
}

void Display::update(const bachelor::Coordinates& msg, const Topic subjTopic)
{
    std::cout << "msg size: " << (int)msg.size << std::endl;
    for(auto i=0; i<msg.size; ++i)
    {
        m_Points[subjTopic].push_back({cv::Point( (msg.X1[i]), (msg.Y1[i]) ), cv::Point( (msg.X2[i]), (msg.Y2[i]) )} );
    }
    m_Recievement[subjTopic] = true;
}

void Display::update(const bachelor::Log& msg, const Topic subjTopic)
{
    if(subjTopic == LogFromECU)
    {
        m_ECULog = msg;
    }
    m_Recievement[subjTopic] = true;
}