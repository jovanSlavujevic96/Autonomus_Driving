#include <bachelor/ECU.hpp>
#include <bachelor/DataSender/DataSender.hpp>

void ECU::assignReceived(const Topic topic)
{
    switch(topic)
    {
        case ECU_LaneDet:
        {
            m_Movement = m_MsgTable[topic];
            break;
        } 
        case ECU_LimDet:
        {
            m_Limit = m_MsgTable[topic];
            break;
        } 
        case ECU_StopDet:
        {
            m_StopDetected = (m_MsgTable[topic] == "STOP") ? true : false;
            break;
        }
    }
}

ECU::ECU() :
    m_LogSender{std::make_unique<DataSender<bachelor::Log>>(LogFromECU) },
    m_ImHere{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_ECU) }
{
    this->m_StopDetected = false;
    this->m_Limit = "NaN";
    this->m_Movement = "NaN";
}

void ECU::addTopic(const Topic topic)
{
    m_MsgTable[topic] = std::string();
}

void ECU::update(const std_msgs::String& msg, const Topic subjTopic)
{
    m_MsgTable[subjTopic] = msg.data;
    ECU::assignReceived(subjTopic);
}

bool ECU::doStuff(void)
{
    {
        std_msgs::Bool msg;
        msg.data = true;
        m_ImHere->Publish(msg);
    }
    {
        bachelor::Log msg;
        if(m_StopDetected)
        {
            m_Movement = "Brake"; 
        }
        msg.movement = m_Movement;
        msg.speed_limit = m_Limit;
        m_LogSender->Publish(msg);
    }
    return true;
}