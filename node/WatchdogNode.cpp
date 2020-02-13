#include <bachelor/Watchdog/Watchdog.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#define loopRate 10

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Watchdog_Node");
    
    std::unique_ptr<IObserver<std_msgs::Bool>> WatchdogObserver = std::make_unique<Watchdog>();
    
    std::unique_ptr<IDataReceiver<std_msgs::Bool>> Subject[NumOfNodes];
    const Topics topics[NumOfNodes] = {fromCAMtoWDOG, fromOBJDETtoWDOG, fromDISPtoWDOG};
    for(int i=0; i<NumOfNodes; ++i)
    {
        Subject[i] = std::make_unique<DataReceiver<std_msgs::Bool> >(topics[i] );
        Subject[i]->registerObserver(WatchdogObserver.get() );
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
