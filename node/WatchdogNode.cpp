#include <bachelor/Watchdog.hpp>
#include <bachelor/DataProtocol/Receiver.hpp>
#include <bachelor/NodeName.h>
#include <bachelor/DataProtocol/PlatformRcvBool.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#define loopRate 10
#define NumOfNodes (7-1) //minus watchdogNode
#define detCluster 3

int main(int argc, char **argv)
{   
    bool watch[3] = {false};
    if(argc >= 2)
    {
        for(int i=1; i<argc; ++i)
        {
            if(!strcmp(argv[i], "yes"))
            {
                watch[i-1] = true;
            }
        }
    }
    const std::string nodeName = "Watchdog_Node";
    ros::init(argc, argv, nodeName); 
    
    Watchdog wdog;

    std::unique_ptr<IReceiver> detectorRcv[detCluster];
    {
        const Topic topics[detCluster] = {ImHere_LaneDet, ImHere_LimDet, ImHere_StopDet};
        for(int i=0; i<detCluster; ++i)
        {
            std::cout <<  NodeName[topics[i]] << ": " << std::boolalpha << watch[i] << std::endl;
            if(watch[i])
            {
                detectorRcv[i] = std::make_unique<Receiver>(std::make_unique<PlatformRcvBool>(topics[i]));
                detectorRcv[i]->registerObserver(&wdog);
                wdog.addNodeToWatch(topics[i]);
            }
        }
    }
    
    const int summaryNodes = NumOfNodes - detCluster;
    std::unique_ptr<IReceiver> msgRcv[summaryNodes];
    const Topic topics[summaryNodes] = {ImHere_CamSim, ImHere_Visual, ImHere_ECU};
    for(int i=0; i<summaryNodes; ++i)
    {
        msgRcv[i] = std::make_unique<Receiver>(std::make_unique<PlatformRcvBool>(topics[i]) );
        msgRcv[i]->registerObserver(&wdog);
        wdog.addNodeToWatch(topics[i]);
    }
    
    std::cout << nodeName << " successfully initialized." << std::endl; 

    ros::Rate loop_rate(loopRate);
    while(ros::ok() && wdog.doStuff() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
