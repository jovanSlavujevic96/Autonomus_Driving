#include <bachelor/Detector/Detector.hpp>

#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <bachelor/ImageProcessor/StopProcessor.hpp>
#include <bachelor/ImageProcessor/SpeedLimitProcessor.hpp>
#include <bachelor/ImageProcessor/RoadLaneProcessor.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ObjectDetector_Node");
	
	std::unique_ptr<IImageProcessor> LaneProc = std::make_unique<RoadLaneProcessor>();
	std::unique_ptr<Detector> ObjDetObserver = std::make_unique<Detector>(std::move(LaneProc) );
	
	std::unique_ptr<IDataReceiver<sensor_msgs::Image>> FrameRcvSubject;
	FrameRcvSubject = std::make_unique<DataReceiver<sensor_msgs::Image> >(fromCAMtoOBJDET);
	FrameRcvSubject->registerObserver(ObjDetObserver.get() );

	/*
	std::unique_ptr<IImageProcessor> LimitProc = std::make_unique<SpeedLimitProcessor>();
	std::unique_ptr<IImageProcessor> LaneProc = std::make_unique<StopProcessor>();
	*/

	while(ros::ok() )
	{
		ObjDetObserver->doStuff();	//send true to watchdog node
		ros::spinOnce(); //spin for new event on topics
	}

	return EXIT_SUCCESS;
}
