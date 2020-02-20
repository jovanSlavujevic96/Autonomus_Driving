#include <bachelor/ImageProcessor/LaneProcessor.hpp>
#include <bachelor/ImageProcessor/StopProcessor.hpp>
#include <bachelor/ImageProcessor/LimitProcessor.hpp>

#include <cv_bridge/cv_bridge.h>

#define play 1
#define pause 0

int main(int argc, char **argv)
{
    std::string VideoPath = "/home/rtrk/Videos/testVideos/LimitTest3.mp4";
    if(argc >= 2)
    {
        VideoPath = std::string("/home/rtrk/Videos/testVideos/") + std::string(argv[1]);
    }

    /*
    LaneProcessor lane;
    cv::VideoCapture cap(VideoPath);
    //*/

    ///*
    LimitProcessor speed;
    cv::VideoCapture cap(VideoPath);
    //*/

    /*
    StopProcessor stop;
    cv::VideoCapture cap(VideoPath);
    //*/

    if(!cap.isOpened() ) 
        return -1;
    
    int state = play;
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    sensor_msgs::Image img1;
    while(cap.isOpened() )
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty() ) break;

        cv_ptr->encoding = "bgr8";
		cv_ptr->image = frame;
		cv_ptr->toImageMsg(img1);

        ///*
        speed.setFrame(img1);
        img1 = speed.getProcessedFrame();
        //*/

        /*
        lane.setFrame(img1);
        img1 = lane.getProcessedFrame();
        //*/

        /*
        stop.setFrame(img1);
        img1 = stop.getProcessedFrame();
        //*/

        frame = cv_bridge::toCvCopy(img1, "bgr8")->image;
		//cv::resize(frame, frame, cv::Size(std::round(frame.cols*0.6), std::round(frame.rows*0.6)));
        cv::imshow("video stream", frame );
        
        //std::cout << speed.getResult() << std::endl;

        auto btn = cv::waitKey(state);
        if(btn == 27 || btn == 'q' || btn == 'Q') break;
        else if(btn == 32 || btn == 'p' || btn == 'P')
        {
            if(state == play) state = pause;
            else state = play;
        }
    }
    return 0;
}