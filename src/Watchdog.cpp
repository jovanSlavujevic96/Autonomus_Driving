#include <bachelor/Watchdog.hpp>
#include <bachelor/NodeName.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <numeric>

#define counter_limit 10
#define mil 1000000

static int count = 0;
static bool key = false;

void Watchdog::initConnection(void)
{
    m_Connection = Watchdog::checkMsgs();
}

bool Watchdog::checkMsgs(void)
{
    for(const auto& rcv : m_ImHereRcv)
    {
        if(!rcv.second)
        {
            return false;
        }
    }
    return true;
}

void Watchdog::checkNodes(void)
{
    if(Watchdog::checkMsgs() && count <= counter_limit)
    {
        std::cout << "\nOK\n";
        count = 0;
        Watchdog::resetMessages(); //reset all messages
    }
    else if(!Watchdog::checkMsgs() && count == counter_limit)
    {
        std::cout << std::endl;
        for(const auto& rcv : m_ImHereRcv)
        {
            if(!rcv.second)
            {
                std::cout << NodeName[rcv.first] << " is unconnected" << std::endl;
            }
        }
        std::cout << std::endl;

        count = 0;
        Watchdog::resetNodes();  //kill all nodes and start again
    } 
}

void Watchdog::resetNodes(void)
{
    std::cout << "Watchdog will reset all nodes." << std::endl;
    usleep(2*mil);
    for(const auto& topic : m_ImHereRcv)
    {
        std::stringstream ss;
        ss << "rosnode kill " << NodeName[topic.first];
        system(ss.str().c_str() );
    }
    system("gnome-terminal --command='roslaunch bachelor project.launch'");

    Watchdog::resetMessages();
    m_Connection = false;  
}

void Watchdog::resetMessages(void)
{
    for(auto& msg : m_ImHereRcv)
    {
        msg.second = false;
    }
}

Watchdog::Watchdog() :
    m_Connection{false}
{
    
}

void Watchdog::addNodeToWatch(const Topic topic)
{
    m_ImHereRcv[topic] = false;
}

void Watchdog::update(const std_msgs::Bool& msg, const Topic subjTopic)
{
    m_ImHereRcv[subjTopic] = msg.data;
    std::cout << "received msg from: " << NodeName[subjTopic] << std::endl;
}

bool Watchdog::doStuff(void)
{
    if( !m_Connection && !key)
    {
        std::cout << "Waiting all nodes to connect." << std::endl;
        key = true;
    }
    else if( m_Connection && key)
    {
        std::cout << "All nodes are connected." << std::endl;
        key = false;
    }

    if( !m_Connection)
    {
        Watchdog::initConnection();
        count = 0;
        return true;
    }
    else
    {
        Watchdog::checkNodes();
        ++count;
        return true; 
    }
}