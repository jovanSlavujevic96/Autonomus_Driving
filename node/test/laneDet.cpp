#include <bachelor/ObjectDetectorNode/RoadLaneProcessor.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define play 30
#define pause 0

int main(void)
{
    RoadLaneProcessor lane;
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    if(!cap.isOpened() ) 
        return -1;
    
    int state = play;
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    sensor_msgs::Image img1;
    while(cap.isOpened() )
    {
        cv::Mat frame;
        cap >> frame;

        cv_ptr->encoding = "bgr8";
		cv_ptr->image = frame;
		cv_ptr->toImageMsg(img1);

        lane.setFrame(img1);
        auto data = lane.getProcessedFrame();

        frame = cv_bridge::toCvCopy(data, "bgr8")->image;
		cv::imshow("video stream", frame );

        auto btn = cv::waitKey(state);
        if(btn == 27 || btn == 'q') break;
        else if(btn == 32 || btn == 'p')
        {
            if(state == play) state = pause;
            else state = play;
        }
    }
    return 0;
}