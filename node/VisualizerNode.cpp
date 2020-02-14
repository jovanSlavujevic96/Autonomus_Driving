#include <ros/ros.h>
#include <string>

#include <bachelor/Visualizer/Visualizer.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

int main(int argc, char **argv)
{
    bool draw[3] = {false};
    if(argc >= 2)
    {
        for(int i=1; i<argc; ++i)
        {
            if(!strcmp(argv[i], "yes"))
            {
                draw[i-1] = true;
            }
        }
    }
    {
        const std::string names[3] = {"lanes", "stop rects", "limit rects"};
        for(int i=0; i<3; ++i)
        {
            std::cout << "draw "<< names[i] << ": " << std::boolalpha << draw[i] << std::endl;
        }
    }
    const std::string nodeName = "Visualizer_Node";
    ros::init(argc, argv, nodeName);

    Visualizer observer(draw[0], draw[1], draw[2]);

    std::unique_ptr<IDataReceiver<bachelor::Coordinates>> coordinatesRcv[3];
    const Topics topics[3] = {Coord_LaneDet, Coord_StopDet, Coord_LimDet};
    for(int i=0; i<3; ++i)
    {
        coordinatesRcv[i] = std::make_unique<DataReceiver<bachelor::Coordinates>>(topics[i]);
        coordinatesRcv[i]->registerObserver(&observer);
    }
    std::unique_ptr<IDataReceiver<sensor_msgs::Image>> framesRcv;
    framesRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(RawFrame);
    framesRcv->registerObserver(&observer);
    

    std::cout << nodeName << " successfully initialized." << std::endl; 

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