#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>
#include <bachelor/ImageProcessor/LimitProcessor.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    const std::string nodeName = "LimitDetector_Node";
	ros::init(argc, argv, nodeName);

    std::unique_ptr<Detector> LimitDetector = std::make_unique<Detector>(std::make_unique<LimitProcessor>(), ImHere_StopDet, Coord_StopDet);

    std::unique_ptr<IDataReceiver<sensor_msgs::Image>> FrameRcv;
	FrameRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(RawFrame);
	FrameRcv->registerObserver(LimitDetector.get() );

    std::cout << nodeName << " successfully initialized." << std::endl;

	while(ros::ok() )
	{
		LimitDetector->doStuff();	//send true to watchdog node
		ros::spinOnce(); //spin for new event on topics
	}

    return 0;
}