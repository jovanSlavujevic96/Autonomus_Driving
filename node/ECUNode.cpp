#include <ros/ros.h>

#include <bachelor/ECU.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

int main(int argc, char **argv)
{
    const std::string nodeName = "ECU_Node";
    ros::init(argc, argv, nodeName);

    ECU ecu;
    std::unique_ptr<IDataReceiver<std_msgs::String>> stringRcv[3];
    const Topic topics[3] = {ECU_LaneDet, ECU_LimDet, ECU_StopDet};
    for(int i=0; i<3; ++i)
    {
        stringRcv[i] = std::make_unique<DataReceiver<std_msgs::String>>(topics[i]);
        stringRcv[i]->registerObserver(&ecu);
        ecu.addTopic(topics[i]);
    }

    std::cout << nodeName << " successfully initialized" << std::endl;
    while(ros::ok() && ecu.doStuff() )
    {
        ros::spinOnce();
    }

    return 0;
}