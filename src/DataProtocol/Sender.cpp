#include <bachelor/DataProtocol/Sender.hpp>
#include <bachelor/TopicName.h>
#include <bachelor/Message/IMessage.hpp>
#include <typeinfo>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <bachelor/Log.h>
#include <bachelor/Coordinates.h>

#include <bachelor/Message/BoolMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>
#include <bachelor/Message/CoordMessage.hpp>

//default
template <typename T1>
class Sender<T1>::ImplDataSender
{
    ros::Publisher Publisher;
public:
    ImplDataSender(const Topic topic)
    {
        ros::NodeHandle node;
        if(typeid(T1) == typeid(bool))
        {
            Publisher = node.advertise<std_msgs::Bool>(TopicName[topic], 1);
        }
        else if(typeid(T1) == typeid(std::string))
        {
            Publisher = node.advertise<std_msgs::String>(TopicName[topic], 1);
        }
        else if(typeid(T1) == typeid(std::vector<std::string>))
        {
            Publisher = node.advertise<bachelor::Log>(TopicName[topic], 1);
        }
        else if(typeid(T1) == typeid(std::vector<std::vector<cv::Point>>))
        {
            Publisher = node.advertise<bachelor::Coordinates>(TopicName[topic], 1);
        }
    };
    virtual ~ImplDataSender() = default;

    void Publish(const IMessage* data)
    {
        if(typeid(T1) == typeid(bool))
        {
            auto Data = static_cast<const BoolMessage*>(data);
            std_msgs::Bool message;
            message.data = Data->info;
            Publisher.publish(message);
        }
        else if(typeid(T1) == typeid(std::string))
        {
            auto Data = static_cast<const StringMessage*>(data);
            if(Data->text.empty() )
            {
                return;
            }
            std_msgs::String message;
            message.data = Data->text[0];
            Publisher.publish(message);
        }
        else if(typeid(T1) == typeid(std::vector<std::string>))
        {
            auto Data = static_cast<const StringMessage*>(data);
            if(Data->text.size() < 2)
            {
                return;
            }
            bachelor::Log message;
            message.movement = Data->text[0];
            message.speed_limit = Data->text[1];
            Publisher.publish(message);
        }
        else if(typeid(T1) == typeid(std::vector<std::vector<cv::Point>>))
        {
            std::cout << "sending coords\n";
            auto Data = static_cast<const CoordMessage*>(data);
            bachelor::Coordinates message;
            message.size = static_cast<uint8_t>(Data->coordinates.size());
            for(int i=0; i<Data->coordinates.size(); ++i)
            {
                message.X1.push_back(Data->coordinates[i][0].x);
                message.Y1.push_back(Data->coordinates[i][0].y);
                message.X2.push_back(Data->coordinates[i][1].x);
                message.Y2.push_back(Data->coordinates[i][1].y);
            }
            Publisher.publish(message);
        }
    };
};

//exception -> frame
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <bachelor/Message/ImageMessage.hpp>

template<>
class Sender<cv::Mat>::ImplDataSender
{
    image_transport::Publisher Publisher;
    cv_bridge::CvImagePtr CVBridge_Converter;
public:
    ImplDataSender(const Topic topic)
    {
        ros::NodeHandle node;
        image_transport::ImageTransport it(node);
        Publisher = it.advertise(TopicName[topic], 1);
        CVBridge_Converter = std::make_unique<cv_bridge::CvImage> ();
    };
    virtual ~ImplDataSender() = default;

    void Publish(const IMessage* data)
    {
        const ImageMessage* Data = static_cast<const ImageMessage*>(data);

		CVBridge_Converter->encoding = "bgr8";
		CVBridge_Converter->image = Data->image;

		sensor_msgs::Image message;
		CVBridge_Converter->toImageMsg(message);
		Publisher.publish(message);
    };
};

template class Sender<cv::Mat>;
template class Sender<std::string>;
template class Sender<std::vector<std::string>>;
template class Sender<std::vector<std::vector<cv::Point>>>;
template class Sender<bool>;

//imp main class
template <typename T1>
Sender<T1>::Sender(const Topic topic) : 
    m_PimplDataSender{std::make_unique<Sender<T1>::ImplDataSender>(topic) }
{

}

template <typename T1>
Sender<T1>::~Sender() = default;

template <typename T1>
void Sender<T1>::Publish(const IMessage* data)
{
    m_PimplDataSender->Publish(data);
}