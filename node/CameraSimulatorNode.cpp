#include <bachelor/CameraSimulator.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>

#include <ros/ros.h>

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

	const std::string nodeName = "CameraSimulator_Node";
	cv::VideoCapture videoCap(videoName);
	if(!videoCap.isOpened() )
	{
		std::cout << "There's no video such as this one:" << std::endl << videoName << std::endl;
		std::cout << "Exit " << nodeName << std::endl;
		return -1;
	}
	else
	{
		std::cout << "Playing video: " << videoName << std::endl;
		std::cout << "Playing fps: " << fps << std::endl;
	}
	
	ros::init(argc, argv, nodeName);

	std::unique_ptr<CameraSimulator> PlayerObserver = std::make_unique<CameraSimulator>();
	PlayerObserver->setVideo(videoCap);

	std::unique_ptr<IDataReceiver<std_msgs::Bool> > DataSubject;
	DataSubject = std::make_unique<DataReceiver<std_msgs::Bool>>(PauseOrPlay);
	DataSubject->registerObserver(PlayerObserver.get());

	std::cout << nodeName << " successfully initialized." << std::endl; 

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
