#include <bachelor/DisplayNode/Display.hpp>
#include <bachelor/DataProtocol/FrameDataReceiver.hpp>

#include <memory>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DisplayNode");
    
    std::unique_ptr<Display> DisplayObserver = std::make_unique<Display>();
    std::unique_ptr<IFrameDataReceiver> FrameRcvSubject = std::make_unique<FrameDataReceiver>(TopicName[fromOBJDETtoDISP] );
    FrameRcvSubject->registerObserver(DisplayObserver.get(), fromOBJDETtoDISP);
    
    while(ros::ok() )
    {
        DisplayObserver->Keyboard();
        ros::spinOnce();
    }
    return 0;
}