#include <bachelor/Detector/Detector.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>
#include <bachelor/ImageProcessor/LaneProcessor.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
	const std::string nodeName = "LaneDetector_Node";
	ros::init(argc, argv, nodeName);
	
	std::unique_ptr<IImageProcessor> LaneProc = std::make_unique<LaneProcessor>();
	std::unique_ptr<Detector> LaneDetector = std::make_unique<Detector>(std::move(LaneProc), ImHere_LaneDet, Coord_LaneDet);
	
	std::unique_ptr<IDataReceiver<sensor_msgs::Image>> FrameRcv;
	FrameRcv = std::make_unique<DataReceiver<sensor_msgs::Image>>(RawFrame);
	FrameRcv->registerObserver(LaneDetector.get() );

    std::cout << nodeName << " successfully initialized." << std::endl;

	while(ros::ok() )
	{
		LaneDetector->doStuff();	//send true to watchdog node
		ros::spinOnce(); //spin for new event on topics
	}

	return EXIT_SUCCESS;
}
