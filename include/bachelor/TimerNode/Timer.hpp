#ifndef BACHELOR_TIMERNODE_TIMER_HPP_
#define BACHELOR_TIMERNODE_TIMER_HPP_

#include <bachelor/Observer/IObserver.hpp>
#include <bachelor/DataProtocol/Template/IDataSender.hpp>

#include <memory>
#include <std_msgs/Bool.h>

template class IObserver<bool>;
class Timer : public IObserver<bool>
{
private:
    std::unique_ptr<IDataSender<bool,std_msgs::Bool> > m_DataEmiterTimerEnds, m_DataEmiterWatchdog;

    bool m_TimerIndicator;
    const unsigned int m_Time;

    bool Count(void);
public:
    Timer(const unsigned int CountLimit);
    virtual ~Timer();

    virtual void update(bool &_data, Topics _subjTopic) override;	//observer method
    virtual bool doStuff(void) override;
};

#endif //BACHELOR_TIMERNODE_TIMER_HPP_