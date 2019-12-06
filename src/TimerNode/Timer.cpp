#include <bachelor/TimerNode/Timer.hpp>
#include <bachelor/DataProtocol/BoolDataEmiter.hpp>

#include <iostream>
#include <chrono>
#include <thread>

bool Timer::Count(void) 
{
	unsigned int milliseconds = 0, seconds = 0;

	while(seconds < m_Time)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(1000) ); 
		++milliseconds;
		if(milliseconds == 200) 
		{
			++seconds;
			milliseconds = 0;
            m_DataEmiterWatchdog->Publish(true);
		}
		system("clear"); 
		std::cout << "\n\n\n\t\t" << "Timer:" <<"| " << seconds << " sec | " << milliseconds/2 << " ms" << std::endl; 
	}

    if(seconds < m_Time)
    {    
        return false;
    }
    return true;
}

Timer::Timer(const unsigned int CountLimit) :
    m_DataEmiterTimerEnds{std::make_unique<BoolDataEmiter>(TopicName[fromTIMERtoVIDEOP] )}, 
    m_DataEmiterWatchdog{std::make_unique<BoolDataEmiter>(TopicName[fromTIMERtoWDOG] )},
    m_TimerIndicator(false), m_Time(CountLimit)
{
    system("clear"); 
    std::cout << "\n\tTimer initialized\n\n";
}

Timer::~Timer()
{
    system("clear"); 
}

void Timer::update(bool _data, Topics _subjTopic)
{
    if(_subjTopic == fromVIDEOPtoTIMER)
    {
        m_TimerIndicator = _data;
    }
    if(m_TimerIndicator)
    {
        if(Timer::Count() );
        {
            m_DataEmiterTimerEnds->Publish(true);
        }
        m_TimerIndicator = false;
    }
}

void Timer::SayHelloToWDog(void)
{
    m_DataEmiterWatchdog->Publish(true);
}