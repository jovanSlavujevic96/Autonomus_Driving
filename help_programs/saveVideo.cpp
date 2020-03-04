#include <opencv2/opencv.hpp>

#define fps 50

#define play 1
#define pause 0

static std::map<bool, int> run_ = { {true, play}, {false, pause} };
static volatile bool state = true;
static cv::VideoCapture input;
static std::unique_ptr<cv::VideoWriter> output;
static cv::Size size;
static cv::Mat frame;
static unsigned int incr=0;

const static bool record = false;

//*
//merge videos

#define inputVideoPath "/home/rtrk/Desktop/videos/"
#define outputVideoPath inputVideoPath "../_FINALvideo.avi"

int main(int argc, char** argv)
{
    {
        std::stringstream ss;
        ss << inputVideoPath << "FINALvideo" << ++incr << ".avi";
        input = cv::VideoCapture(ss.str() );
    }
    input >> frame;
    size = frame.size() ;
    if(record)
    {
        output = std::make_unique<cv::VideoWriter>(outputVideoPath, CV_FOURCC('M','J','P','G'), fps, size );
    }

    while(true)
    {
        if(size.width > 0 && size.height > 0)
        {
            cv::resize(frame, frame, size);   
        }
        cv::imshow("frame", frame);
        if(record)
        {
            *output << frame;
        }
        auto btn = cv::waitKey(run_[(bool)state]);
        if(btn == 27 || btn == 'q')
        {
            break;
        }
        else if(btn == 32 || btn == 'p')
        {
            state = !state;
        }
        input >> frame;
        if(frame.empty() )
        {
            input.release();
            std::stringstream ss;
            ss << inputVideoPath << "FINALvideo" << ++incr << ".avi";
            if(incr == 3)
            {
                break;
            }
            input = cv::VideoCapture(ss.str() );
            input >> frame;
        }
    }
    if(output.get() != NULL && record)
    {
        output->release();
    }
    return 0;
}
//*/

/*
//cut videos

#define inputVideoPath "/home/rtrk/Desktop/detectionVideo2.avi" 
#define outputVideoPath "/home/rtrk/Desktop/"

int main(int argc, char **argv)
{
    input = cv::VideoCapture(inputVideoPath);//(0);
    if(!input.isOpened() )
    {
        std::cout << "wrong video\n";
        return -1;
    }
    input >> frame;
    size = frame.size();
    {
        std::stringstream ss;
        ss << outputVideoPath << "cut" << incr++ << ".avi"; 
        output = std::make_unique<cv::VideoWriter>(ss.str(),CV_FOURCC('M','J','P','G'),fps, size );
    }
    
    while(!frame.empty() )
    {   
        cv::imshow("frame", frame);
        *output << frame;
        auto btn = cv::waitKey(run_[(bool)state]);
        if(btn == 27 || btn == 'q')
        {
            break;
        }
        else if(btn == 32 || btn == 'p')
        {
            state = !state;
        }
        else if(btn == 'c')
        {
            output.release();  
            std::stringstream ss;
            ss << outputVideoPath << "cut" << incr++ << ".avi"; 
            output = cv::VideoWriter(ss.str(),CV_FOURCC('M','J','P','G'),fps, size); 
        }
        input >> frame;
    }
    input.release();
    output->release();

    return 0;
}
//*/