#include <bachelor/DataProtocol/DataSender.hpp>

#include <ros/ros.h>


//relevant types
#include <image_transport/image_transport.h>
template class DataSender<sensor_msgs::Image,sensor_msgs::Image>;

#include <std_msgs/Bool.h>
template class DataSender<bool, std_msgs::Bool>; 

#include <std_msgs/Int64.h>
template class DataSender<int, std_msgs::Int64>;

#include <std_msgs/String.h>
#include <string>
template class DataSender<std::string, std_msgs::String>;


//default
template <typename T1, typename T2>
class DataSender<T1,T2>::ImplDataSender
{
    ros::Publisher Publisher;
public:
    ImplDataSender(Topics _topicName)
    {
        ros::NodeHandle node;
        Publisher = node.advertise<T2>(TopicName[_topicName], 1);
    };
    virtual ~ImplDataSender() = default;

    void Publish(T1 _data)
    {
        T2 message;
        message.data = _data;
        Publisher.publish(message);
    };
};

//special case -> frame
template<>
class DataSender<sensor_msgs::Image,sensor_msgs::Image>::ImplDataSender
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

    void Publish(sensor_msgs::Image _data)
    {
        Publisher.publish(_data);
    };
};

//imp main class
template <typename T1, typename T2>
DataSender<T1,T2>::DataSender(Topics _topicName) : 
    m_PimplDataSender{std::make_unique<DataSender<T1,T2>::ImplDataSender>(_topicName) }
{

}

template <typename T1, typename T2>
DataSender<T1,T2>::~DataSender() = default;

template <typename T1, typename T2>
void DataSender<T1,T2>::Publish(T1 _data)
{
    m_PimplDataSender->Publish(_data);
}