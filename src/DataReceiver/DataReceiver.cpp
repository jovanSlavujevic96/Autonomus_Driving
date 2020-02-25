#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <ros/ros.h>
#include <vector>
#include <bachelor/TopicName.h>

//relevant types
#include <image_transport/image_transport.h>
template class DataReceiver<sensor_msgs::Image>;

#include <std_msgs/Bool.h>
template class DataReceiver<std_msgs::Bool>; 

#include <bachelor/Coordinates.h>
template class DataReceiver<bachelor::Coordinates>;

#include <std_msgs/String.h>
template class DataReceiver<std_msgs::String>;

#include <bachelor/Log.h>
template class DataReceiver<bachelor::Log>;

//default
template <typename T1>
class DataReceiver<T1>::ImplDataReceiver
{
    ros::Subscriber Subscriber;
    Topic TopicTo;
    std::vector<IObserver<T1>*> Observers;

    void Callback(const T1& msg)
    {
        notifyObservers(msg);
    };
    void notifyObservers(const T1& data)
    {
        for (IObserver<T1>* observer : Observers)  // notify all observers
        {
            observer->update(data, TopicTo);
        }
    };
public:
    ImplDataReceiver(const Topic topicName) : 
        TopicTo{topicName}
    {
        ros::NodeHandle node;
        Subscriber = node.subscribe(TopicName[topicName], 1, &DataReceiver<T1>::ImplDataReceiver::Callback, this);    
    };
	~ImplDataReceiver() = default;

    void registerObserver(IObserver<T1>* observer)
    {
        Observers.push_back(observer);
    };
	void removeObserver(IObserver<T1>* observer)
    {
        auto iterator = std::find(Observers.begin(), Observers.end(), observer);
        if (iterator != Observers.end()) // observer found
        {
            Observers.erase(iterator); // remove the observer
        }
    };
};


//exception -> frame
#include <cv_bridge/cv_bridge.h>
#include <iostream>

template <>
class DataReceiver<sensor_msgs::Image>::ImplDataReceiver
{
    image_transport::Subscriber Subscriber;
    Topic TopicTo;
    std::vector<IObserver<sensor_msgs::Image>*> Observers;

    void Callback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            notifyObservers(*msg);
        }
        catch(cv_bridge::Exception& error)
        {
            std::cerr << "Error receiving message" << std::endl;
        }
    };
    void notifyObservers(const sensor_msgs::Image& data)
    {
        for (IObserver<sensor_msgs::Image>* observer : Observers)  // notify all observers
        {
            observer->update(data, TopicTo);
        }
    };
public:
    ImplDataReceiver(const Topic topicName) : TopicTo{topicName}
    {
        ros::NodeHandle node;
        image_transport::ImageTransport it(node);
        Subscriber = it.subscribe(TopicName[topicName], 1, &DataReceiver<sensor_msgs::Image>::ImplDataReceiver::Callback, this);   
    };
	~ImplDataReceiver() = default;

    void registerObserver(IObserver<sensor_msgs::Image>* observer)
    {
        Observers.push_back(observer);
    };
	void removeObserver(IObserver<sensor_msgs::Image>* observer)
    {
        auto iterator = std::find(Observers.begin(), Observers.end(), observer);
        if (iterator != Observers.end()) // observer found
        {
            Observers.erase(iterator); // remove the observer
        }
    };
};

//main class impl
template <typename T1>
DataReceiver<T1>::DataReceiver(const Topic topicName) :
    m_PimplDataReceiver{std::make_unique<DataReceiver<T1>::ImplDataReceiver>(topicName) }
{

}

template <typename T1>
DataReceiver<T1>::~DataReceiver() = default;

template <typename T1>
void DataReceiver<T1>::registerObserver(IObserver<T1>* observer)
{
    m_PimplDataReceiver->registerObserver(observer);
}

template <typename T1>
void DataReceiver<T1>::removeObserver(IObserver<T1>* observer)
{
    m_PimplDataReceiver->removeObserver(observer);
}