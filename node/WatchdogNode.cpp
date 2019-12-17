#include <bachelor/WatchdogNode/Watchdog.hpp>
#include <bachelor/DataProtocol/DataReceiver.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#define loopRate 10

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WatchdogNode");
    
    std::unique_ptr<IObserver<bool> > WatchdogObserver = std::make_unique<Watchdog>();
    
    std::unique_ptr<IDataReceiver<bool,std_msgs::Bool> > Subject[NumOfNodes];
    const Topics topics[NumOfNodes] = {fromVIDEOPtoWDOG, fromOBJDETtoWDOG, fromTIMERtoWDOG, fromDISPtoWDOG};
    for(int i=0; i<NumOfNodes; ++i)
    {
        Subject[i] = std::make_unique<DataReceiver<bool,std_msgs::Bool> >(topics[i] );
        Subject[i]->registerObserver(WatchdogObserver.get(), topics[i]);
    }
    
    ros::Rate loop_rate(loopRate);
    while(ros::ok() )
    {
        WatchdogObserver->doStuff();
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return EXIT_SUCCESS;
}
