#include <bachelor/WatchdogNode/Watchdog.hpp>
#include <bachelor/DataProtocol/BoolDataReceiver.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>
#define loopRate 10

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WatchdogNode");
    
    std::unique_ptr<Watchdog> WatchdogObserver = std::make_unique<Watchdog>();
    
    std::unique_ptr<IBoolDataReceiver> Subject[NumOfNodes];
    const Topics topics[NumOfNodes] = {fromVIDEOPtoWDOG, fromOBJDETtoWDOG, fromTIMERtoWDOG, fromDISPtoWDOG};
    for(int i=0; i<NumOfNodes; ++i)
    {
        Subject[i] = std::make_unique<BoolDataReceiver>(TopicName[topics[i]] );
        Subject[i]->registerObserver(WatchdogObserver.get(), topics[i]);
    }
    
    ros::Rate loop_rate(loopRate);
    while(ros::ok() )
    {
        WatchdogObserver->DoStuff();
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return EXIT_SUCCESS;
}
