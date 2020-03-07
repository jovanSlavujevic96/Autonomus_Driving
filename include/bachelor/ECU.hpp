#ifndef BACHELOR_ECU_HPP_
#define BACHELOR_ECU_HPP_

#include <bachelor/IObserver.hpp>
#include <bachelor/DataProtocol/ISender.hpp>
#include <memory>

class ECU :
    public IObserver
{
    std::map<Topic, std::string> m_MsgTable;
    std::unique_ptr<ISender> m_LogSender;
    std::unique_ptr<ISender> m_ImHere;

    bool m_StopDetected;
    std::string m_Movement;
    std::string m_Limit;

    void assignReceived(const Topic topic);
public:
    ECU();
    virtual ~ECU() = default;

    void update(const IPlatformRcv* receiver) override;
    bool doStuff(void) override;
};

#endif//BACHELOR_ECU_HPP_