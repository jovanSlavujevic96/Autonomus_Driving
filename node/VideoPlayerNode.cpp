#include <bachelor/VideoPlayerNode/VideoPlayer.hpp>
#include <bachelor/DataProtocol/BoolDataReceiver.hpp>

#include <iostream>
#include <sstream>

#include <ros/ros.h>

#define videoLib "/home/rtrk/Videos/"

void programStartChecking(int argc, char **argv, std::string &videoName);
void typeVideo(cv::VideoCapture &video, std::string &videoName);

int main(int argc, char **argv)
{
	std::string videoName;
	programStartChecking(argc, argv, videoName);

	cv::VideoCapture videoCap;
	typeVideo(videoCap, videoName);

	ros::init(argc, argv, "VideoPlayerNode");
	
	std::unique_ptr<VideoPlayer> PlayerObserver = std::make_unique<VideoPlayer>();
	PlayerObserver->setVideo(videoCap);
	
	std::unique_ptr<IBoolDataReceiver> DataSubject[3];
	const Topics topics[3] = {fromTIMERtoVIDEOP, fromDISPtoVIDEOP, fromOBJDETtoVIDEOP};
	for(int i=0; i<3; ++i)
	{
		DataSubject[i] = std::make_unique<BoolDataReceiver>(TopicName[topics[i]] );
		DataSubject[i]->registerObserver(PlayerObserver.get(), topics[i]);
	}

	std::cout << std::endl << "\tSending raw video frames." << std::endl << std::endl; 

	while(ros::ok() )
	{
		bool info = PlayerObserver->Cycle();
		if(!info)
		{
			videoName.erase();
			typeVideo(videoCap, videoName);
			PlayerObserver->setVideo(videoCap);
		}
		cv::waitKey(30); //60 za sporije
		ros::spinOnce();
	}
	return EXIT_SUCCESS;
}

void programStartChecking(int argc, char **argv, std::string &videoName)
{
	if(argc == 2)
	{
		videoName = argv[1];
		std::cout << "You entered the video file named: " << videoName << '.' << std::endl << std::endl;
	}
	else if(argc < 2)
	{
		std::cout << "You didn't enter the video file name. " << std::endl;
		std::cout << "Enter the video file name: ";
		std::cin >> videoName;
		std::cout << std::endl;
	}
	else
	{
		std::cout << "There's too much arguments" << std::endl << std::endl;
		std::exit(-1);
	}
}

cv::VideoCapture getVideo(const std::string videoName, bool &error)
{
	cv::VideoCapture videoCap;
	std::stringstream ss;
	ss << videoLib << videoName;
	videoCap = cv::VideoCapture(ss.str() );
	bool info = videoCap.isOpened();
	std::cout << "Video Capture is opened: " << std::boolalpha << info << std::endl;
	if(info)
	{
		error = false;
		return videoCap;
	}
	std::cout << "Cannot find video: " << videoName << std::endl;
	std::cout << "In this directory: " << videoLib << '.' << std::endl << std::endl;
	error = true;
	return videoCap;	
}

void typeVideo(cv::VideoCapture &video, std::string &videoName)
{
	bool error = true;
	while(error)
	{ 
		video = getVideo(videoName, error);
		if(error)
		{
			std::cout << "Enter new video: ";
			std::cin >> videoName;
			std::cout << std::endl;
		}
	}
}