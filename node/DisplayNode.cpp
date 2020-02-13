#include <bachelor/Display/Display.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Display_Node");
    
    std::unique_ptr<IObserver<sensor_msgs::Image> > DisplayObserver = std::make_unique<Display>();

    std::unique_ptr<IDataReceiver<sensor_msgs::Image>> FrameRcvSubject; 
    FrameRcvSubject = std::make_unique<DataReceiver<sensor_msgs::Image>>(fromOBJDETtoDISP);
    FrameRcvSubject->registerObserver(DisplayObserver.get() );
    
    while(ros::ok() )
    {
        DisplayObserver->doStuff();
        ros::spinOnce();
    }
    return 0;
}