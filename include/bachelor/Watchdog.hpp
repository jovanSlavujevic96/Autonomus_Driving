#ifndef BACHELOR_WATCHDOG_HPP_
#define BACHELOR_WATCHDOG_HPP_

#include <bachelor/IObserver.hpp>

#include <string>
#include <map>
#include <std_msgs/Bool.h>

#define NumOfNodes 3

class Watchdog : 
    public IObserver<std_msgs::Bool>
{
private:
    std::map<Topics, bool*> m_TopicMap;
    std::map<bool*, std::string> m_NodeMap;

    bool m_NodeMSG[NumOfNodes];
    bool m_Connection;

    void initMaps(void);
    void initConnection(void);
    void resetNodes(void);
    void resetMessages(void);
    void checkNodes(void);

public:
    Watchdog();
    virtual ~Watchdog() = default;

    void update(const std_msgs::Bool& msg, Topics subjTopic) override;
    bool doStuff(void) override;
};

#endif //BACHELOR_WATCHDOG_HPP_