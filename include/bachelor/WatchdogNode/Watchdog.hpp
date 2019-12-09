#ifndef BACHELOR_WATCHDOGNODE_WATCHDOG_HPP_
#define BACHELOR_WATCHDOGNODE_WATCHDOG_HPP_

#include <bachelor/Observer/IObserver.hpp>

#include <string>
#include <map>

#define NumOfNodes 4

template class IObserver<bool>;
class Watchdog : public IObserver<bool>
{
private:
    std::map<Topics, int> m_TopicMap;
    std::map<bool *, std::string> m_NodeMap;

    bool m_NodeMSG[NumOfNodes];
    bool m_Connection;

    void initMaps(void);
    void initConnection(void);
    void resetNodes(void);
    void checkNodes(void);

public:
    Watchdog();
    virtual ~Watchdog();

    virtual void update(bool &_data, Topics _subjTopic) override;
    virtual bool doStuff(void) override;
};

#endif //BACHELOR_WATCHDOGNODE_WATCHDOG_HPP_