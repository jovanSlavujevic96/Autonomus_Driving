#include <opencv2/opencv.hpp>

#define play 30
#define pause 0

#define saveDir "/home/rtrk/Desktop/saved_frames/"

int main(void)
{
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    if(!cap.isOpened() ) return -1;
    cv::Mat frame;
    int state = play;
    unsigned int incr = 0;
    while(cap.isOpened() )
    {
        cap >> frame;
        //cv::resize(frame, frame, cv::Size( std::round(frame.rows*0.6), std::round(frame.cols*0.6) ));
        cv::imshow("stream", frame);
        auto btn = cv::waitKey(state);  
        if(btn == 27 || btn == 'q') break;
        else if(btn == 32 || btn == 'p')
        {
            if(state == play) 
            {
                state = pause;
                incr++;
                std::stringstream ss;
                ss << saveDir << "frame " << incr << ".jpg";
                cv::imwrite(ss.str(), frame);
            }
            else state = play;
        }
    }
    cap.release();
    return 0;
}