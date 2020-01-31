#include <bachelor/ObjectDetectorNode/ObjectDetector.hpp>

#include <bachelor/DataProtocol/DataReceiver.hpp>
//#include <image_transport/image_transport.h>

#include <bachelor/ObjectDetectorNode/StopSignProcessor.hpp>
#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>
#include <bachelor/ObjectDetectorNode/RoadLaneProcessor.hpp>

#include <bachelor/Topics.h>

#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ObjectDetectorNode");
	
	std::unique_ptr<ObjectDetector> ObjDetObserver = std::make_unique<ObjectDetector>();
	
	std::unique_ptr<IDataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr> > FrameRcvSubject;
	FrameRcvSubject = std::make_unique<DataReceiver<sensor_msgs::Image,sensor_msgs::ImageConstPtr> >(fromVIDEOPtoOBJDET);
	FrameRcvSubject->registerObserver(ObjDetObserver.get(), fromVIDEOPtoOBJDET );

	std::unique_ptr<IImageProcessor> StopProc = std::make_unique<StopSignProcessor>();
	ObjDetObserver->addImageProcessor(StopProc.get() );

	//std::unique_ptr<IImageProcessor> LimitProc = std::make_unique<SpeedLimitProcessor>();
	//ObjDetObserver->addImageProcessor(LimitProc.get() );

	//std::unique_ptr<IImageProcessor> LaneProc = std::make_unique<RoadLaneProcessor>();
	//ObjDetObserver->addImageProcessor(LaneProc.get() );

	while(ros::ok() )
	{
		ObjDetObserver->doStuff();	//send true to watchdog node
		ros::spinOnce(); //spin for new event on topics
	}
	
	return EXIT_SUCCESS;
}
