#include <bachelor/ObjectDetectorNode/RoadLaneProcessor.hpp>
#include <bachelor/ObjectDetectorNode/StopSignProcessor.hpp>
#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define play 30
#define pause 0

int main(void)
{
    ///*
    RoadLaneProcessor lane;
    //cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    //*/

    ///*
    SpeedLimitProcessor speed;
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/OUTFILE-1.mp4");
    //*/

    /*
    StopSignProcessor stop;
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/drivingSchool.mp4");
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

        ///*
        lane.setFrame(img1);
        img1 = lane.getProcessedFrame();
        //*/

        /*
        stop.setFrame(img1);
        img1 = stop.getProcessedFrame();
        //*/

        frame = cv_bridge::toCvCopy(img1, "bgr8")->image;
		cv::imshow("video stream", frame );

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