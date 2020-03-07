#include <bachelor/Display.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataProtocol/Sender.hpp>
#include <bachelor/Frame.h>
#include <bachelor/Visualizer/LogVisualizer.hpp>
#include <bachelor/DataProtocol/IPlatformRcv.hpp>
#include <bachelor/Message/IMessage.hpp>
#include <bachelor/Message/BoolMessage.hpp>
#include <bachelor/Message/ImageMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>
#include <bachelor/Message/CoordMessage.hpp>

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
        m_Points[rcv.first].clear();
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

        BoolMessage msg;
        msg.info = m_Pause;
        m_PauseSender->Publish(&msg); //publish message to CameraSimulator_Node
    }
    return true;
}

void Display::assignStringAndPoints(std::vector<std::string>*& text, std::vector<std::vector<cv::Point>>*& pts, const Topic topic)
{
    pts = &m_Points[topic];
}

bool Display::drawAndDisplay(void)
{
    if(Display::calculateRecievement() && !m_Frame.empty() )
    {
        if(!m_Pause)
        {
            BoolMessage msg;
            msg.info = false;
            m_PauseSender->Publish(&msg);
        }
        Frame frame;
        std::vector<std::string> text;
        frame.MatFrame = &m_Frame;
        frame.Text = &m_Log;
        for(auto& visualizer : m_Visualizers)
        {
            Display::assignStringAndPoints(frame.Text, frame.Dots, visualizer.first);
            visualizer.second->draw(frame);
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
    m_PauseSender{std::make_unique<Sender<bool>>(PauseOrPlay)},
    m_ImHere{std::make_unique<Sender<bool>>(ImHere_Visual)},
    m_Log{std::vector<std::string>(2, "NaN")}
{
    this->m_Pause = false;
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
        BoolMessage msg;
        msg.info = true;
        m_ImHere->Publish(&msg);
    }
    return Display::drawAndDisplay();	//if exit button (<esc> or 'q') is pressed exit the program
}

static int a=0;

void Display::update(const IPlatformRcv* receiver) 
{
    {
        auto msg = static_cast<const ImageMessage*>(receiver->getMessage());
        if(msg->topic == RawFrame)
        {
            m_Frame = msg->image.clone();
            if(!m_Pause)
            {
                BoolMessage msg;
                msg.info = true;
                m_PauseSender->Publish(&msg);
            }
            m_Recievement[msg->topic] = true;
            std::cout << "frame rcv\n";
            return;
        }
    }
    {    auto msg = static_cast<const StringMessage*>(receiver->getMessage());
        if(msg->topic == LogFromECU )
        {
            m_Recievement[msg->topic] = true;
            if(msg->text.size() < 2 || m_Log.size() < 2)
            {
                return;
            }
            for(int i=0; i<2; ++i)
            {
                m_Log[i] = msg->text[i];
            }
            return;
        }
    }
    {
        auto msg = static_cast<const CoordMessage*>(receiver->getMessage());
        if(msg->topic == Coord_LaneDet || msg->topic == Coord_LimDet || msg->topic == Coord_StopDet )
        {
            for(auto i=0; i<msg->coordinates.size(); ++i)
            {
                m_Points[msg->topic] = msg->coordinates;
            }
            m_Recievement[msg->topic] = true;
        }
    }  
}
