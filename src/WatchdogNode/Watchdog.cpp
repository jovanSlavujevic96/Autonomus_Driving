#include <bachelor/WatchdogNode/Watchdog.hpp>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <numeric>

#define counter_limit 200
#define mil 1000000

static int count = 0;
static bool key = false;

void Watchdog::initMaps(void)
{
    const Topics topics[NumOfNodes] = {fromVIDEOPtoWDOG, fromOBJDETtoWDOG, fromTIMERtoWDOG, fromDISPtoWDOG };
    for(int i=0; i<NumOfNodes; ++i)
        m_TopicMap[topics[i]] = i;
    
    const std::string nodes[NumOfNodes] = {"VideoPlayerNode", "ObjectDetectorNode", "TimerNode", "DisplayNode"};
    for(int i=0; i<NumOfNodes; ++i)
        m_NodeMap[&m_NodeMSG[i]] = nodes[i];
}

void Watchdog::initConnection(void)
{
    if(std::accumulate(m_NodeMSG, m_NodeMSG + NumOfNodes, 0) == NumOfNodes)
    {
        m_Connection = true;
        return;
    }
    m_Connection = false;
}

void Watchdog::checkNodes(void)
{
    if(std::accumulate(m_NodeMSG, m_NodeMSG + NumOfNodes, 0) == NumOfNodes && count <= counter_limit)
    {
        std::cout << "\nOK\n";
        count = 0;
        std::fill(std::begin(m_NodeMSG), std::end(m_NodeMSG), false); //reset all msgs
    }
    else if(std::accumulate(m_NodeMSG, m_NodeMSG + NumOfNodes, 0) < NumOfNodes  && count == counter_limit)
    {
        std::cout << std::endl;
        for(int i=0; i<NumOfNodes; ++i)
            if(!m_NodeMSG[i])
                std::cout << m_NodeMap[&m_NodeMSG[i]] << " is unconnected" << std::endl;
        std::cout << std::endl;

        std::cout << "NOT OK\n";
        std::cout << "Press <enter> to proceed." << std::endl;
        std::cin.get();

        count = 0;
        Watchdog::resetNodes();  //kill all nodes and start again
    } 
}

void Watchdog::resetNodes(void)
{
    std::cout << "Watchdog will reset all nodes." << std::endl;
    usleep(2*mil);
    system("clear");
    for(int i=0; i<NumOfNodes; ++i)
    {
        if(m_NodeMSG[i])
        {
            std::stringstream ss;
            ss << "rosnode kill " << m_NodeMap[&m_NodeMSG[i]];
            system(ss.str().c_str() );
        }
    }
    system("rosnode cleanup");
    usleep(2*mil);
    system("clear");
    system("gnome-terminal --command='roslaunch bachelor final.launch'");
    usleep(2*mil);
    system("clear");

    std::fill(std::begin(m_NodeMSG), std::end(m_NodeMSG), false); //reset all msgs
    m_Connection = false;  
}

Watchdog::Watchdog() : m_NodeMSG{false}
{
    Watchdog::initMaps();    
    system("clear");
    std::cout << "\n\tWatchdog initialized\n\n";
}

Watchdog::~Watchdog()
{
    system("clear");
}

void Watchdog::update(bool _data, Topics _subjTopic)
{
    m_NodeMSG[m_TopicMap[_subjTopic]] = _data;
}

void Watchdog::DoStuff(void)
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
    }
    else
    {
        Watchdog::checkNodes();
        ++count; 
    }
}