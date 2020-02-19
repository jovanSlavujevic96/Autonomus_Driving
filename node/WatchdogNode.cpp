#include <bachelor/Watchdog.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#define loopRate 10

int main(int argc, char **argv)
{   
    const std::string nodeName = "Watchdog_Node";
    ros::init(argc, argv, nodeName); 
    
    std::unique_ptr<IObserver<std_msgs::Bool>> wdog = std::make_unique<Watchdog>();
    
    std::unique_ptr<IDataReceiver<std_msgs::Bool>> msgRcv[NumOfNodes];
    const Topics topics[NumOfNodes] = {ImHere_CamSim, ImHere_Visual, ImHere_LaneDet};
    for(int i=0; i<NumOfNodes; ++i)
    {
        msgRcv[i] = std::make_unique<DataReceiver<std_msgs::Bool> >(topics[i] );
        msgRcv[i]->registerObserver(wdog.get() );
    }
    
    std::cout << nodeName << " successfully initialized." << std::endl; 

    ros::Rate loop_rate(loopRate);
    while(ros::ok() )
    {
        wdog->doStuff();
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return EXIT_SUCCESS;
}
