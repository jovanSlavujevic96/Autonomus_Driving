#include <bachelor/CameraSimulator/CameraSimulator.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <iostream>
#include <sstream>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#define videoLib "/home/rtrk/Videos/testVideos/"

void programStartChecking(int argc, char **argv, std::string &videoName);
void typeVideo(cv::VideoCapture &video, std::string &videoName);

int main(int argc, char **argv)
{
	std::string videoName = std::string(videoLib) + std::string("LimitTest2.mp4");
	int fps = 5;
	if(argc >= 2)
	{
		videoName = std::string(videoLib) + std::string(argv[1]);
	}
	if(argc >= 3)
	{
		fps = std::stoi(argv[2]);
	}

	const std::string NodeName = "CameraSimulator_Node";
	cv::VideoCapture videoCap(videoName);
	if(!videoCap.isOpened() )
	{
		std::cout << "There's no video such as this one:" << std::endl << videoName << std::endl;
		std::cout << "Exit node: " << NodeName << std::endl;
		return -1;
	}
	ros::init(argc, argv, NodeName);

	std::unique_ptr<CameraSimulator> PlayerObserver = std::make_unique<CameraSimulator>();
	PlayerObserver->setVideo(videoCap);

	std::unique_ptr<IDataReceiver<std_msgs::Bool> > DataSubject;
	DataSubject = std::make_unique<DataReceiver<std_msgs::Bool> >(fromDISPtoCAM);
	DataSubject->registerObserver(PlayerObserver.get());

	std::cout << std::endl << "\tSending raw video frames." << std::endl << std::endl; 

	while(ros::ok() )
	{
		bool info = PlayerObserver->doStuff();
		if(!info)
		{
			std::cout << "No more frames!" << std::endl;
			break;
		}
		cv::waitKey(fps); //60 za sporije
		ros::spinOnce();
	}

	return EXIT_SUCCESS;
}
