#include <bachelor/ECU.hpp>
#include <bachelor/DataProtocol/Sender.hpp>
#include <vector>
#include <bachelor/DataProtocol/IPlatformRcv.hpp>
#include <bachelor/Message/IMessage.hpp>
#include <bachelor/Message/BoolMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>


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
    m_LogSender{std::make_unique<Sender<std::vector<std::string>>>(LogFromECU) },
    m_ImHere{std::make_unique<Sender<bool>>(ImHere_ECU) }
{
    this->m_StopDetected = false;
    this->m_Limit = "NaN";
    this->m_Movement = "NaN";
}

void ECU::update(const IPlatformRcv* receiver)
{
    auto msg = static_cast<const StringMessage*>(receiver->getMessage());
    m_MsgTable[msg->topic] = msg->text[0];
    ECU::assignReceived(msg->topic);
}

bool ECU::doStuff(void)
{
    {
        BoolMessage msg;
        msg.info = true;
        m_ImHere->Publish(&msg);
    }
    {
        StringMessage msg;
        msg.text = std::vector<std::string>(2);
        if(m_StopDetected)
        {
            m_Movement = "Brake"; 
        }
        msg.text[0] = m_Movement;
        msg.text[1] = m_Limit;
        m_LogSender->Publish(&msg);
    }
    return true;
}