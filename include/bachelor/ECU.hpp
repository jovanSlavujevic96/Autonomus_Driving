#ifndef BACHELOR_ECU_HPP_
#define BACHELOR_ECU_HPP_

#include <bachelor/IObserver.hpp>
#include <std_msgs/String.h>

class ECU :
    public IObserver<std_msgs::String>
{
public:
    ECU();
    virtual ~ECU() = default;

    void update(const std_msgs::String& msg, Topics subjTopic) override;
    bool doStuff(void) override;
};

#endif//BACHELOR_ECU_HPP_