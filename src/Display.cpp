#include <bachelor/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <bachelor/Frame.h>
#include <bachelor/Visualizer/LogVisualizer.hpp>

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
        case LogFromECU:
        {
            text.push_back(m_ECULog.movement);
            text.push_back(m_ECULog.speed_limit);
            return;
        }
    }
    pts = m_Points[topic];
    m_Points[topic].clear();
}

bool Display::drawAndDisplay(void)
{
    if(Display::calculateRecievement() && !m_Frame.empty() )
    {
        if(!m_Pause)
        {
            std_msgs::Bool msg;
            msg.data = false;
            m_PauseSender->Publish(msg);
        }
        Frame frame;
        frame.MatFrame = &m_Frame;
        for(auto& visualizer : m_Visualizers)
        {
            if(visualizer.first == LogFromECU)
            {
                continue;
            }
            Display::assignStringAndPoints(*frame.Text, *frame.Dots, visualizer.first);
            visualizer.second->draw(frame);
        }
        //it must be the last one
        if(m_Visualizers[LogFromECU] != NULL)
        {
            Display::assignStringAndPoints(*frame.Text, *frame.Dots, LogFromECU);
            m_Visualizers[LogFromECU]->draw(frame);
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

Display::Display() : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(PauseOrPlay)},
    m_ImHere{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_Visual)}
{
    this->m_Pause = false;

    this->m_ECULog.speed_limit = "NaN";
    this->m_ECULog.movement = "NaN";
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
    return Display::drawAndDisplay();	//if exit button (<esc> or 'q') is pressed exit the program
}

void Display::update(const sensor_msgs::Image& msg, const Topic subjTopic) 
{
    if(subjTopic == RawFrame)
    {
        m_Frame = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
        if(!m_Pause)
        {
            std_msgs::Bool msg;
            msg.data = true;
            m_PauseSender->Publish(msg);
        }
    }
    m_Recievement[subjTopic] = true;
}

void Display::update(const bachelor::Coordinates& msg, const Topic subjTopic)
{
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