#include <ros/ros.h>

#include <bachelor/TimerNode/Timer.hpp>
#include <bachelor/DataProtocol/DataReceiver.hpp>
#include <bachelor/Topics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TimerNode");

    std::unique_ptr<IObserver<bool> > TimerObserver = std::make_unique<Timer>(3);

    std::unique_ptr<IDataReceiver<bool,std_msgs::Bool> > Subject;
    Subject = std::make_unique<DataReceiver<bool,std_msgs::Bool> >(fromVIDEOPtoTIMER);

    Subject->registerObserver(TimerObserver.get(), fromVIDEOPtoTIMER );

    bool info = true;
    while(ros::ok() && info)
    {
        info = TimerObserver->doStuff();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
