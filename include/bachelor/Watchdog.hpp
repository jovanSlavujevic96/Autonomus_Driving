#ifndef BACHELOR_WATCHDOG_HPP_
#define BACHELOR_WATCHDOG_HPP_

#include <bachelor/IObserver.hpp>

#include <string>
#include <map>
#include <vector>

class Watchdog : 
    public IObserver
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

    void update(const IPlatformRcv* receiver) override;
    bool doStuff(void) override;
};

#endif //BACHELOR_WATCHDOG_HPP_