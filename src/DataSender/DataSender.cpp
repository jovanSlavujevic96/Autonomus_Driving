#include <bachelor/DataSender/DataSender.hpp>

#include <ros/ros.h>


//relevant types
#include <image_transport/image_transport.h>
template class DataSender<sensor_msgs::Image>;

#include <std_msgs/Bool.h>
template class DataSender<std_msgs::Bool>;

#include <bachelor/Coordinates.h>
template class DataSender<bachelor::Coordinates>;


//default
template <typename T1>
class DataSender<T1>::ImplDataSender
{
    ros::Publisher Publisher;
public:
    ImplDataSender(Topics _topicName)
    {
        ros::NodeHandle node;
        Publisher = node.advertise<T1>(TopicName[_topicName], 1);
    };
    virtual ~ImplDataSender() = default;

    void Publish(T1 &_data)
    {
        Publisher.publish(_data);
    };
};

//exception -> frame
template<>
class DataSender<sensor_msgs::Image>::ImplDataSender
{
    image_transport::Publisher Publisher;
public:
    ImplDataSender(Topics _topicName)
    {
        ros::NodeHandle node;
        image_transport::ImageTransport it(node);
        Publisher = it.advertise(TopicName[_topicName], 1);
    };
    virtual ~ImplDataSender() = default;

    void Publish(sensor_msgs::Image &_data)
    {
        Publisher.publish(_data);
    };
};


//imp main class
template <typename T1>
DataSender<T1>::DataSender(Topics _topicName) : 
    m_Topic{_topicName} , 
    m_PimplDataSender{std::make_unique<DataSender<T1>::ImplDataSender>(_topicName) }
{

}

template <typename T1>
DataSender<T1>::~DataSender() = default;

template <typename T1>
void DataSender<T1>::Publish(T1 &_data)
{
    m_PimplDataSender->Publish(_data);
}

template <typename T1>
Topics DataSender<T1>::getTopic(void) const
{
    return m_Topic;
}