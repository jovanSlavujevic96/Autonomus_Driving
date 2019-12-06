#ifndef BACHELOR_TIMERNODE_TIMER_HPP_
#define BACHELOR_TIMERNODE_TIMER_HPP_

#include <bachelor/Observer/IBoolObserver.hpp>
#include <bachelor/DataProtocol/IBoolDataEmiter.hpp>

#include <memory>

class Timer : public IBoolObserver
{
private:
    std::unique_ptr<IBoolDataEmiter> m_DataEmiterTimerEnds, m_DataEmiterWatchdog;

    bool m_TimerIndicator;
    const unsigned int m_Time;

    bool Count(void);
public:
    Timer(const unsigned int CountLimit);
    virtual ~Timer();

    virtual void update(bool _data, Topics _subjTopic) override;	//observer method
    void SayHelloToWDog(void);
};

#endif //BACHELOR_TIMERNODE_TIMER_HPP_