#include <bachelor/Watchdog.hpp>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <numeric>

#define counter_limit 10
#define mil 1000000

static int count = 0;
static bool key = false;

void Watchdog::initMaps(void)
{
    const Topics topics[NumOfNodes] = {ImHere_CamSim, ImHere_Visual, ImHere_LaneDet };
    const std::string nodes[NumOfNodes] = {"CameraSimulator_Node", "Visualizer_Node", "LaneDetector_Node"};
    for(int i=0; i<NumOfNodes; ++i)
    {
        m_TopicMap[topics[i]] = &m_NodeMSG[i];
        m_NodeMap[&m_NodeMSG[i]] = nodes[i];
    }
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
        Watchdog::resetMessages(); //reset all messages
    }
    else if(std::accumulate(m_NodeMSG, m_NodeMSG + NumOfNodes, 0) < NumOfNodes  && count == counter_limit)
    {
        std::cout << std::endl;
        for(int i=0; i<NumOfNodes; ++i)
        {
            if(!m_NodeMSG[i])
            {
                std::cout << m_NodeMap[&m_NodeMSG[i]] << " is unconnected" << std::endl;
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
    for(int i=0; i<NumOfNodes; ++i)
    {
        std::stringstream ss;
        ss << "rosnode kill " << m_NodeMap[&m_NodeMSG[i]];
        system(ss.str().c_str() );
    }
    system("gnome-terminal --command='roslaunch bachelor project.launch'");

    Watchdog::resetMessages();
    m_Connection = false;  
}

void Watchdog::resetMessages(void)
{
    std::fill(std::begin(m_NodeMSG), std::end(m_NodeMSG), false); //reset all msgs
}

Watchdog::Watchdog() : m_NodeMSG{false}
{
    Watchdog::initMaps();    
}

void Watchdog::update(const std_msgs::Bool &_msg, Topics _subjTopic)
{
    *m_TopicMap[_subjTopic] = _msg.data;
    std::cout << "received msg from: " << m_NodeMap[m_TopicMap[_subjTopic]] << std::endl;
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