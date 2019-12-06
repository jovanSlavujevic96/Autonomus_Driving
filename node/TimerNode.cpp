#include <ros/ros.h>
#include <bachelor/TimerNode/Timer.hpp>
#include <bachelor/DataProtocol/BoolDataReceiver.hpp>
#include <bachelor/Topics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TimerNode");

    std::unique_ptr<IBoolObserver> TimerObserver = std::make_unique<Timer>(3);
    std::unique_ptr<IBoolDataReceiver> Subject = std::make_unique<BoolDataReceiver>(TopicName[fromVIDEOPtoTIMER] );
    Subject->registerObserver(TimerObserver.get(), fromVIDEOPtoTIMER );

    ros::spin();

    return EXIT_SUCCESS;
}
