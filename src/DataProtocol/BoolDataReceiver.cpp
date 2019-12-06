#include <bachelor/DataProtocol/BoolDataReceiver.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class BoolDataReceiver::ImplDataReceiver
{
    ros::Subscriber Subscriber;
    void Callback(const std_msgs::Bool &BoolMsg);

    Topics Topic;
    std::vector<IBoolObserver *> observers;
    void notifyObservers(bool _data);

public:
    ImplDataReceiver(const std::string _topicName);
	~ImplDataReceiver() = default;

    void registerObserver(IBoolObserver *observer, Topics _subjTopic);
	void removeObserver(IBoolObserver *observer);
};

void BoolDataReceiver::ImplDataReceiver::notifyObservers(bool _data)
{
    for (IBoolObserver *observer : observers)  // notify all observers
	{    
		observer->update(_data, Topic);
	}
}

void BoolDataReceiver::ImplDataReceiver::Callback(const std_msgs::Bool &BoolMsg)
{
    BoolDataReceiver::ImplDataReceiver::notifyObservers(BoolMsg.data);
}

BoolDataReceiver::ImplDataReceiver::ImplDataReceiver(const std::string _topicName)
{
    ros::NodeHandle nh;
    Subscriber = nh.subscribe(_topicName, 1, &BoolDataReceiver::ImplDataReceiver::Callback, this);
}

void BoolDataReceiver::ImplDataReceiver::registerObserver(IBoolObserver *observer, Topics _subjTopic)
{
    observers.push_back(observer);
    Topic = _subjTopic;
}

void BoolDataReceiver::ImplDataReceiver::removeObserver(IBoolObserver *observer)
{
    auto iterator = std::find(observers.begin(), observers.end(), observer);
    if (iterator != observers.end()) // observer found
	{
        observers.erase(iterator); // remove the observer
    }
}

BoolDataReceiver::BoolDataReceiver(const std::string _topicName) : m_PimplDataReceiver{std::make_unique< BoolDataReceiver::ImplDataReceiver>(_topicName) }
{

}

BoolDataReceiver::~BoolDataReceiver() = default;

void BoolDataReceiver::registerObserver(IBoolObserver *observer, Topics _subjTopic)
{
    m_PimplDataReceiver->registerObserver(observer, _subjTopic);
}

void BoolDataReceiver::removeObserver(IBoolObserver *observer)
{
    m_PimplDataReceiver->removeObserver(observer);
}