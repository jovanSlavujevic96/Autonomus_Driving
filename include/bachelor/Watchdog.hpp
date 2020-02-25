#ifndef BACHELOR_WATCHDOG_HPP_
#define BACHELOR_WATCHDOG_HPP_

#include <bachelor/IObserver.hpp>

#include <string>
#include <map>
#include <vector>
#include <std_msgs/Bool.h>

class Watchdog : 
    public IObserver<std_msgs::Bool>
{
private:
    std::map<Topic, bool> m_ImHereRcv;
    bool m_Connection;

    void initConnection(void);
    void resetNodes(void);
    void resetMessages(void);
    bool checkMsgs(void);
    void checkNodes(void);

public:
    Watchdog();
    virtual ~Watchdog() = default;

    void addNodeToWatch(const Topic topic);

    void update(const std_msgs::Bool& msg, const Topic subjTopic) override;
    bool doStuff(void) override;
};

#endif //BACHELOR_WATCHDOG_HPP_