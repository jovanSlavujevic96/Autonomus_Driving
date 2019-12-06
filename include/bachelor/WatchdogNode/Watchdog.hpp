#ifndef BACHELOR_WATCHDOG_NODE_WATCHDOG_HPP_
#define BACHELOR_WATCHDOG_NODE_WATCHDOG_HPP_

#include <bachelor/Observer/IBoolObserver.hpp>

#include <string>
#include <map>

#define NumOfNodes 3

class Watchdog : public IBoolObserver
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

    virtual void update(bool _data, Topics _subjTopic) override;
    bool getConnection(void) const;	
};

#endif //BACHELOR_WATCHDOG_NODE_WATCHDOG_HPP_