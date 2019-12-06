#include <bachelor/Watchdog_node/Watchdog.hpp>
#include <bachelor/DataProtocol/BoolDataReceiver.hpp>

#include <memory>
#include <string>
#include <iostream>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Watchdog_node");
    
    std::unique_ptr<Watchdog> WatchdogObserver = std::make_unique<Watchdog>();
    
    std::unique_ptr<IBoolDataReceiver> Subject[3];
    const Topics topics[NumOfNodes] = {fromVIDEOPtoWDOG, fromOBJDETtoWDOG, fromTIMERtoWDOG};
    for(int i=0; i<NumOfNodes; ++i)
    {
        Subject[i] = std::make_unique<BoolDataReceiver>(TopicName(topics[i]) );
        Subject[i]->registerObserver(WatchdogObserver.get(), topics[i]);
    }

    std::cout << "Waiting for nodes" << std::endl;

    while( ! WatchdogObserver->getConnection() )
        ros::spinOnce();

    std::cout << "All nodes are connected" << std::endl;
    
    ros::spin();

    return EXIT_SUCCESS;
}
