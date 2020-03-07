#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCV_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCV_HPP_

#include "IPlatformRcv.hpp"
#include <vector>
#include <ros/ros.h>
#include <bachelor/Topic.h>

class PlatformRcv : public IPlatformRcv
{
protected:
    std::vector<IObserver *> m_Observers;
    ros::Subscriber m_Sub;
    void notifyObservers(void);

public:
    virtual void registerObserver(IObserver* observer) override;
	virtual void removeObserver(IObserver* observer) override;

    virtual const IMessage* getMessage(void) const = 0;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCV_HPP_