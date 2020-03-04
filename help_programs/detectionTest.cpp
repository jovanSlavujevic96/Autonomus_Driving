#include <bachelor/ImageProcessor/LaneProcessor.hpp>
#include <bachelor/ImageProcessor/StopProcessor.hpp>
#include <bachelor/ImageProcessor/LimitProcessor.hpp>

#include <bachelor/Visualizer/LogVisualizer.hpp>

#include <cv_bridge/cv_bridge.h>

#define play 10
#define pause 0
#define inputVideoPath "/home/rtrk/Videos/testVideos/LimitTest3.mp4"
#define outputVideoPath "/home/rtrk/Desktop/detection_output.avi"
#define savetImagesPath "/home/rtrk/Desktop/linije2/"
#define fps 30

#include <memory>

const static bool record = false;
const static bool trackSTOP = false;
const static bool trackLANE = true; //false;
const static bool trackLIMIT = true; //false;

void allocateImageProcessor(std::unique_ptr<LaneProcessor>& lane, std::unique_ptr<LimitProcessor>& speed, std::unique_ptr<StopProcessor>& stop)
{
    if(trackLANE)
    {
        lane = std::make_unique<LaneProcessor>(CamCalSolution1);
    }
    if(trackLIMIT)
    {
        speed = std::make_unique<LimitProcessor>();
    }
    if(trackSTOP)
    {
        stop = std::make_unique<StopProcessor>();
    }
}

int main(int argc, char **argv)
{
    std::string VideoPath = inputVideoPath;
    if(argc >= 2)
    {
        VideoPath = std::string("/home/rtrk/Videos/testVideos/") + std::string(argv[1]);
    }

    std::unique_ptr<LaneProcessor> lane;
    std::unique_ptr<LimitProcessor> speed;
    std::unique_ptr<StopProcessor> stop;
    allocateImageProcessor(lane,speed,stop);

    cv::VideoCapture cap(VideoPath);
    if(!cap.isOpened() ) 
    {
        std::cout << std::endl << "Theres no such a video: " << std::endl << VideoPath << std::endl;
        return -1;
    }
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
    sensor_msgs::Image img1;

    LogVisualizer logViz;
    cv::Mat frame;
    cap >> frame;
    Frame frame_;
    frame_.MatFrame = &frame;

    std::map<bool, int> run_;
    run_[true] = play;
    run_[false] = pause;
    bool state = true;

    std::vector<std::string> text(2);
    for(int i=0; i<2; ++i)
    {
        text[i] = "NaN";
    }
    frame_.Text = &text;

    cv::VideoWriter video;  
    if(record)
    {
        video = cv::VideoWriter(outputVideoPath,CV_FOURCC('M','J','P','G'),fps, frame.size());
    }
    while(cap.isOpened() && !frame.empty() )
    {
		cv_ptr->image = frame;
		cv_ptr->toImageMsg(img1);

        if(trackLIMIT)
        {
            speed->setFrame(img1);
            img1 = speed->getProcessedFrame();
            text[1] = speed->getResult();
        }
        if(trackLANE)
        {
            lane->setFrame(img1);
            img1 = lane->getProcessedFrame();
            auto tmp = 
            text[0] = lane->getResult();
        }
        if(trackSTOP)
        {
            stop->setFrame(img1);
            img1 = stop->getProcessedFrame();
            if( (stop->getResult() == "STOP" && trackLANE) || !trackLANE)
            {
                text[0] = stop->getResult();
            }
        }
        frame = cv_bridge::toCvCopy(img1, "bgr8")->image.clone();

        logViz.draw(frame_);
        if(record)
        {
            video << frame; //write in processed frame into the video
        }
        cv::resize(frame, frame, cv::Size(std::round(frame.cols*0.6), std::round(frame.rows*0.6)));
        cv::imshow("video stream", frame );

        auto btn = cv::waitKey(run_[state]);
        if(btn == 27 || btn == 'q' || btn == 'Q') 
        {
            break;
        }
        else if(btn == 32 || btn == 'p' || btn == 'P')
        {
            state = !state;
        }
        cap >> frame;
    }
    cap.release();
    video.release();
    cv::destroyAllWindows();

    return 0;
}