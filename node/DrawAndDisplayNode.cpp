#include <ros/ros.h>
#include <string>

#include <bachelor/DrawAndDisplay/DrawAndDisplay.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

int main(int argc, char **argv)
{
    bool drawLanes = false;
    if(argc >= 2)
    {
        if(!strcmp(argv[1], "yes"))
        {
            drawLanes = true;
        }
        else
        {
            drawLanes = false;
        }
    }
    const std::string NodeName = "DrawAndDisplay_Node";
    ros::init(argc, argv, NodeName);

    std::unique_ptr<IDataReceiver<bachelor::Coordinates>> coordinatesRcv;
    coordinatesRcv = std::make_unique<DataReceiver<bachelor::Coordinates>>(fromOBJDETtoDRAW);

    std::unique_ptr<IDataReceiver<sensor_msgs::Image>> framesRcv;
    framesRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(fromCAMtoOBJDET);

    DrawAndDisplay observer(drawLanes);
    framesRcv->registerObserver(&observer);
    coordinatesRcv->registerObserver(&observer);

    while(ros::ok() )
    {
        ros::spinOnce();
        if( !observer.doStuff() ) 
        {
            break;
        }
    }

    return 0;
}