#include <bachelor/DataSender/DataSender.hpp>
#include <bachelor/TopicName.h>

#include <ros/ros.h>

//relevant types
#include <image_transport/image_transport.h>
template class DataSender<sensor_msgs::Image>;

#include <std_msgs/Bool.h>
template class DataSender<std_msgs::Bool>;

#include <bachelor/Coordinates.h>
template class DataSender<bachelor::Coordinates>;

#include <std_msgs/String.h>
template class DataSender<std_msgs::String>;

#include <bachelor/Log.h>
template class DataSender<bachelor::Log>;


//default
template <typename T1>
class DataSender<T1>::ImplDataSender
{
    ros::Publisher Publisher;
public:
    ImplDataSender(const Topic topicName)
    {
        ros::NodeHandle node;
        Publisher = node.advertise<T1>(TopicName[topicName], 1);
    };
    virtual ~ImplDataSender() = default;

    void Publish(const T1& data)
    {
        Publisher.publish(data);
    };
};

//exception -> frame
template<>
class DataSender<sensor_msgs::Image>::ImplDataSender
{
    image_transport::Publisher Publisher;
public:
    ImplDataSender(const Topic topicName)
    {
        ros::NodeHandle node;
        image_transport::ImageTransport it(node);
        Publisher = it.advertise(TopicName[topicName], 1);
    };
    virtual ~ImplDataSender() = default;

    void Publish(const sensor_msgs::Image& data)
    {
        Publisher.publish(data);
    };
};


//imp main class
template <typename T1>
DataSender<T1>::DataSender(const Topic topicName) : 
    m_Topic{topicName} , 
    m_PimplDataSender{std::make_unique<DataSender<T1>::ImplDataSender>(topicName) }
{

}

template <typename T1>
DataSender<T1>::~DataSender() = default;

template <typename T1>
void DataSender<T1>::Publish(const T1& data)
{
    m_PimplDataSender->Publish(data);
}

template <typename T1>
Topic DataSender<T1>::getTopic(void) const
{
    return m_Topic;
}