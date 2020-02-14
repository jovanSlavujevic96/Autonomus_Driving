#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>
#include <bachelor/ImageProcessor/StopProcessor.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
	const std::string nodeName = "StopDetector_Node";
	ros::init(argc, argv, nodeName);
	
	std::unique_ptr<Detector> StopDetector = std::make_unique<Detector>(std::make_unique<StopProcessor>(), ImHere_StopDet, Coord_StopDet);
	
	std::unique_ptr<IDataReceiver<sensor_msgs::Image>> FrameRcv;
	FrameRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(RawFrame);
	FrameRcv->registerObserver(StopDetector.get() );

    std::cout << nodeName << " successfully initialized." << std::endl;

	while(ros::ok() )
	{
		StopDetector->doStuff();	//send true to watchdog node
		ros::spinOnce(); //spin for new event on topics
	}

	return EXIT_SUCCESS;
}
