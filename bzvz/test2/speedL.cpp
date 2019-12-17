#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>
#include <memory>
#include <cv_bridge/cv_bridge.h> 
#include <sstream>

//rosrun bachelor LimitProc video.mp4
#define VideoLib "/home/rtrk/Videos/"

//rosrun bachelor LimitProc image.jpg
#define PictureLib "/home/rtrk/Pictures/"

int main(int argc, char **argv)
{
    //cv::VideoCapture videoCap;
    cv::Mat INPUTframe;
    if(argc == 2)
    {
        std::stringstream ss;
        ss << PictureLib << argv[1];
        //videoCap = cv::VideoCapture(ss.str() );
        INPUTframe = cv::imread(ss.str() );
        if(INPUTframe.empty() )
        {
            std::cout << "There's no image such as: " << argv[1] << std::endl;
            std::cout << "in this directory: " << ss.str() << std::endl;
            return EXIT_FAILURE;
        }
    }
    else
    {
        std::cout << "Exit program." << std::endl;
        return EXIT_FAILURE;
    }

    std::unique_ptr<IImageProcessor> processor;
    processor = std::make_unique<SpeedLimitProcessor>();

    char btn = 1;
    int state = 1;
    //while(btn != 27)
    {
        cv::Mat InputFrame /*;//*/= INPUTframe.clone();
        //videoCap >> InputFrame;

        cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = InputFrame;
        sensor_msgs::Image img1;
        cv_ptr->toImageMsg(img1);
        processor->setFrame(img1);
        cv::Mat OutputFrame = cv_bridge::toCvCopy(processor->getProcessedFrame(), "bgr8")->image.clone();

        cv::imshow("frames", OutputFrame);
        btn = cv::waitKey(/*state*/);
        /*
        if(btn == 32)
        {
            if(state) 
                state = 0;
            else 
                state = 30; 
        }
        */
    }
    
    
    return EXIT_SUCCESS;
}