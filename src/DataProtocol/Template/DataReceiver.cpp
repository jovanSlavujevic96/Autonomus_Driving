#include <bachelor/DataProtocol/Template/DataReceiver.hpp>

#include <ros/ros.h>
#include <vector>

//relevant types
#include <image_transport/image_transport.h>
template class DataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr>;

#include <std_msgs/Bool.h>
template class DataReceiver<bool, std_msgs::Bool>; 

#include <std_msgs/Int64.h>
template class DataReceiver<int, std_msgs::Int64>;

#include <std_msgs/String.h>
#include <string>
template class DataReceiver<std::string, std_msgs::String>;


//default
template <typename T1, typename T2>
class DataReceiver<T1,T2>::ImplDataReceiver
{
    ros::Subscriber Subscriber;
    Topics TopicTo;
    std::vector<IObserver<T1> *> Observers;

    void Callback(const T2 &_Msg)
    {
        notifyObservers(_Msg.data);
    };
    void notifyObservers(T1 _data)
    {
        for (IObserver<T1> *observer : Observers)  // notify all observers
            observer->update(_data, TopicTo);
    };
public:
    ImplDataReceiver(const Topics _topicName)
    {
        ros::NodeHandle node;
        Subscriber = node.subscribe(TopicName[_topicName], 1, &DataReceiver<T1,T2>::ImplDataReceiver::Callback, this);    
    };
	~ImplDataReceiver() = default;

    void registerObserver(IObserver<T1> *observer, Topics _subjTopic)
    {
        Observers.push_back(observer);
        TopicTo = _subjTopic;
    };
	void removeObserver(IObserver<T1> *observer)
    {
        auto iterator = std::find(Observers.begin(), Observers.end(), observer);
        if (iterator != Observers.end()) // observer found
            Observers.erase(iterator); // remove the observer
    };
};


//special case -> frame
#include <cv_bridge/cv_bridge.h>
#include <iostream>

template <>
class DataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr>::ImplDataReceiver
{
    image_transport::Subscriber Subscriber;
    Topics TopicTo;
    std::vector<IObserver<sensor_msgs::Image> *> Observers;

    void Callback(const sensor_msgs::ImageConstPtr &_Msg)
    {
        try
        {
            notifyObservers(*_Msg);
        }
        catch(cv_bridge::Exception &error)
        {
            std::cerr << "Error receiving message" << std::endl;
        }
    };
    void notifyObservers(sensor_msgs::Image _data)
    {
        for (IObserver<sensor_msgs::Image> *observer : Observers)  // notify all observers
            observer->update(_data, TopicTo);
    };
public:
    ImplDataReceiver(const Topics _topicName)
    {
        ros::NodeHandle node;
        image_transport::ImageTransport it(node);
        Subscriber = it.subscribe(TopicName[_topicName], 1, &DataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr>::ImplDataReceiver::Callback, this);   
    };
	~ImplDataReceiver() = default;

    void registerObserver(IObserver<sensor_msgs::Image> *observer, Topics _subjTopic)
    {
        Observers.push_back(observer);
        TopicTo = _subjTopic;
    };
	void removeObserver(IObserver<sensor_msgs::Image> *observer)
    {
        auto iterator = std::find(Observers.begin(), Observers.end(), observer);
        if (iterator != Observers.end()) // observer found
            Observers.erase(iterator); // remove the observer
    };
};


//main class impl
template <typename T1, typename T2>
DataReceiver<T1,T2>::DataReceiver(const Topics _topicName) :
    m_PimplDataReceiver{std::make_unique<DataReceiver<T1,T2>::ImplDataReceiver>(_topicName) }
{

}

template <typename T1, typename T2>
DataReceiver<T1,T2>::~DataReceiver() = default;

template <typename T1, typename T2>
void DataReceiver<T1,T2>::registerObserver(IObserver<T1> *observer, Topics _subjTopic)
{
    m_PimplDataReceiver->registerObserver(observer, _subjTopic);
}

template <typename T1, typename T2>
void DataReceiver<T1,T2>::removeObserver(IObserver<T1> *observer)
{
    m_PimplDataReceiver->removeObserver(observer);
}