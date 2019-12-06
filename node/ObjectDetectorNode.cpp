#include <bachelor/ObjectDetectorNode/ObjectDetector.hpp>
#include <bachelor/DataProtocol/FrameDataReceiver.hpp>
#include <bachelor/ObjectDetectorNode/StopSignProcessor.hpp>

#include <bachelor/Topics.h>

#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ObjectDetectorNode");
	
	std::unique_ptr<ObjectDetector> ObjDetObserver = std::make_unique<ObjectDetector>();
	std::unique_ptr<IFrameDataReceiver> FrameRcvSubject = std::make_unique<FrameDataReceiver>(TopicName[fromVIDEOPtoOBJDET] );
	std::unique_ptr<IImageProcessor> StopProc = std::make_unique<StopSignProcessor>();
	ObjDetObserver->addImageProcessor(StopProc.get() );
	StopSignProcessor StopProc2;
	ObjDetObserver->addImageProcessor(&StopProc2);

	FrameRcvSubject->registerObserver(ObjDetObserver.get(), fromVIDEOPtoOBJDET );

	while(ros::ok() )
	{
		ObjDetObserver->sendDataToTopic(fromOBJDETtoWDOG, true);	//send true to watchdog node
		
		ros::spinOnce(); //spin for new event on topics
	}
	
	return EXIT_SUCCESS;
}
