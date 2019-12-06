#include <bachelor/Watchdog_node/Watchdog.hpp>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <numeric>

#define counter_limit 100
#define mil 1000000

static int count = 0;

void Watchdog::initMaps(void)
{
    const Topics topics[NumOfNodes] = {fromVIDEOPtoWDOG, fromOBJDETtoWDOG, fromTIMERtoWDOG };
    for(int i=0; i<NumOfNodes; ++i)
        m_TopicMap[topics[i]] = i;
    
    const std::string nodes[NumOfNodes] = {"VideoPlayer_node", "ObjectDetector_node", "Timer_node"};
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
                std::cout << m_NodeMap[&m_NodeMSG[i]] << " unconenected" << std::endl;
        std::cout << std::endl;

        std::cout << "NOT OK\n";
        std::cout << "Press <enter> to proceed." << std::endl;
        std::cin.get();

        count = 0;
        Watchdog::resetNodes();  //kill all nodes and start again
        std::fill(std::begin(m_NodeMSG), std::end(m_NodeMSG), false); //reset all msgs
    } 
    ++count; 
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
    
    if(! m_Connection)
        Watchdog::initConnection();
    else
        Watchdog::checkNodes();
}

bool Watchdog::getConnection(void) const
{
    return m_Connection;
}