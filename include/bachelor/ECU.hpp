#ifndef BACHELOR_ECU_HPP_
#define BACHELOR_ECU_HPP_

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

//messages
#include <std_msgs/String.h>
#include <bachelor/Log.h>
#include <std_msgs/Bool.h>

class ECU :
    public IObserver<std_msgs::String>
{
    std::map<Topic, std::string> m_MsgTable;
    std::unique_ptr<IDataSender<bachelor::Log>> m_LogSender;
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_ImHere;

    bool m_StopDetected;
    std::string m_Movement;
    std::string m_Limit;

    void assignReceived(const Topic topic);
public:
    ECU();
    virtual ~ECU() = default;

    void addTopic(const Topic topic);

    void update(const std_msgs::String& msg, const Topic subjTopic) override;
    bool doStuff(void) override;
};

#endif//BACHELOR_ECU_HPP_