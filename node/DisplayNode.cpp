#include <bachelor/DisplayNode/Display.hpp>
#include <bachelor/DataProtocol/DataReceiver.hpp>

#include <memory>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DisplayNode");
    
    std::unique_ptr<IObserver<sensor_msgs::Image> > DisplayObserver = std::make_unique<Display>();

    std::unique_ptr<IDataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr> > FrameRcvSubject; 
    FrameRcvSubject = std::make_unique<DataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr> >(fromOBJDETtoDISP);
    FrameRcvSubject->registerObserver(DisplayObserver.get(), fromOBJDETtoDISP);
    
    while(ros::ok() )
    {
        DisplayObserver->doStuff();
        ros::spinOnce();
    }
    return 0;
}